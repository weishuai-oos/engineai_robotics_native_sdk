#include "rl_supermimic/rl_supermimic_runner.h"

#include <glog/logging.h>
#include <filesystem>
#include <map>
#include <set>
#include <stdexcept>

#include "math/roll_pitch_yaw.h"
#include "math/rotation_matrix.h"
#include "parameter/global_config_initializer.h"

namespace runner {
namespace {

std::string SemanticName(const std::string& name) {
  const auto delimiter = name.find('_');
  if (delimiter == std::string::npos) throw std::runtime_error("Invalid SuperMimic joint name: " + name);
  return name.substr(delimiter + 1);
}

bool FiniteVector(const Eigen::VectorXd& values, bool nonnegative = false) {
  return values.size() == supermimic::kActions && values.allFinite() &&
         (!nonnegative || (values.array() >= 0.0).all());
}

}  // namespace

void RlSupermimicRunner::ValidateParameters() {
  if (param_->schema_version != 1 || param_->method != "supermimic" || param_->robot != "t800" ||
      param_->history_length != supermimic::kHistory || param_->observation_dim != supermimic::kObservation ||
      param_->goal_dim != supermimic::kGoal || param_->input_dim != supermimic::kInput ||
      param_->output_dim != supermimic::kActions || param_->history_order != "oldest_to_newest_frame_major" ||
      param_->deployment_precision != "float32")
    throw std::runtime_error("Unsupported SuperMimic export contract");
  if (!std::isfinite(param_->control_dt) || std::abs(param_->control_dt - 0.02) > 1e-9 ||
      !std::isfinite(param_->physics_dt) || std::abs(param_->physics_dt - 0.002) > 1e-9 ||
      (runner_period_ > 0 && std::abs(runner_period_ - param_->control_dt) > 1e-9))
    throw std::runtime_error("SuperMimic requires a 0.02s runner period and 0.002s physics step");
  if (!FiniteVector(param_->default_joint_pos) || !FiniteVector(param_->action_scale, true) ||
      !FiniteVector(param_->joint_stiffness, true) || !FiniteVector(param_->joint_damping, true) ||
      !FiniteVector(param_->effort_limits, true) ||
      (param_->imu_install_bias && !param_->imu_install_bias->allFinite()))
    throw std::runtime_error("Invalid SuperMimic joint parameters");
}

void RlSupermimicRunner::BuildJointMapping() {
  if (param_->joint_names.size() != supermimic::kActions ||
      param_->mujoco_joint_names.size() != supermimic::kActions ||
      model_param_->num_total_joints != supermimic::kActions)
    throw std::runtime_error("SuperMimic T800 expects exactly 25 joints");
  std::map<std::string, std::string> names;
  for (const auto& name : param_->mujoco_joint_names)
    if (!names.emplace(SemanticName(name), name).second)
      throw std::runtime_error("Duplicate SuperMimic SDK joint semantics");
  std::set<int> indices;
  mapping_.resize(supermimic::kActions);
  for (int i = 0; i < supermimic::kActions; ++i) {
    const auto& sdk_name = names.at(SemanticName(param_->joint_names[i]));
    const int index = model_param_->joint_id_in_total_limb.at(sdk_name);
    if (!indices.insert(index).second || index < 0 || index >= model_param_->num_total_joints)
      throw std::runtime_error("Duplicate or invalid SuperMimic deployed joint");
    mapping_(i) = index;
  }
}

void RlSupermimicRunner::ReadState(Eigen::Matrix3d* rotation, Eigen::Vector3d* body_angular_velocity) {
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, actual_q_);
  data_store_->joint_info.GetState(data::JointInfoType::kVelocity, actual_qd_);
  const auto imu = data_store_->imu_info.Get();
  if (!imu || actual_q_.size() != model_param_->num_total_joints || actual_qd_.size() != actual_q_.size() ||
      !actual_q_.allFinite() || !actual_qd_.allFinite() || !imu->quaternion.coeffs().allFinite() ||
      imu->quaternion.norm() < 1e-8 || !imu->angular_velocity.allFinite())
    throw std::runtime_error("Invalid SuperMimic measured state");
  const Eigen::Matrix3d imu_rotation = math::RotationMatrixd(imu->quaternion).matrix();
  *rotation = imu_rotation * installation_.transpose();
  *body_angular_velocity = rotation->transpose() * imu_rotation * imu->angular_velocity;
}

bool RlSupermimicRunner::Enter() {
  try {
    param_ = data::ParamManager::create<data::RlSupermimicParam>(
        param_tag_.empty() ? "rl_supermimic_come_here" : param_tag_);
    if (!param_) throw std::runtime_error("Cannot load SuperMimic parameters");
    ValidateParameters();
    BuildJointMapping();
    const auto package = std::filesystem::path(common::GlobalPathManager::GetInstance().GetConfigPath()) /
                         std::filesystem::path(param_->GetScope()).parent_path();
    const auto asset = [&](const std::string& path) {
      if (path.empty() || std::filesystem::path(path).is_absolute())
        throw std::runtime_error("SuperMimic asset must be a nonempty package-relative path");
      const auto result = package / path;
      if (!std::filesystem::is_regular_file(result))
        throw std::runtime_error("Missing SuperMimic asset: " + result.string());
      return result.string();
    };
    reference_ = supermimic::LoadReference(asset(param_->reference_file));
    policy_ = std::make_unique<supermimic::FlatPolicy>(asset(param_->policy_file));
    if (!sequence_.Configure(reference_.frames(), param_->hold_seconds, param_->control_dt))
      throw std::runtime_error("Invalid SuperMimic playback duration");
    const Eigen::Vector3d bias = param_->imu_install_bias.value_or(Eigen::Vector3d::Zero());
    installation_ = math::RotationMatrixd(math::RollPitchYawd(bias)).matrix();
    Eigen::Matrix3d rotation;
    Eigen::Vector3d angular_velocity;
    ReadState(&rotation, &angular_velocity);
    yaw_alignment_ = supermimic::Yaw(rotation) * supermimic::Yaw(reference_.rotations.front()).transpose();
    previous_action_ = Eigen::VectorXd::Zero(supermimic::kActions);
    first_observation_ = true;
    live_command_.Resize(model_param_->num_total_joints);
    live_command_.q = actual_q_;
    live_command_.kp(mapping_) = param_->joint_stiffness;
    live_command_.kd(mapping_) = param_->joint_damping;
    command_ = live_command_;

    motion_transition::EntryTransitionConfig config;
    config.enabled = param_->entry_transition_enabled.value_or(true);
    config.nominal_duration = param_->entry_transition_duration.value_or(0.20);
    config.min_duration = param_->entry_transition_min_duration.value_or(std::min(0.10, config.nominal_duration));
    config.max_duration = param_->entry_transition_max_duration.value_or(std::max(0.28, config.nominal_duration));
    config.max_joint_velocity = param_->entry_transition_max_joint_velocity.value_or(8.0);
    config.max_joint_acceleration = param_->entry_transition_max_joint_acceleration.value_or(120.0);
    // The reference can include a raised leg: blend the live policy output,
    // without adding a separate PD transition to the reference joint pose.
    config.reference_pose_weight = 0.0;
    config.source_command_tracking_error = param_->entry_transition_source_tracking_error.value_or(0.75);
    motion_transition::JointCommand source;
    motion_transition::CaptureJointCommand(data_store_->joint_info, model_param_->num_total_joints, &source);
    if (!transition_.Configure(config) || !transition_.Start(source, actual_q_, actual_qd_, live_command_))
      throw std::runtime_error("Cannot initialize SuperMimic entry bridge");
    GetMutableOutput().Reset();
    SetRunnerState(RunnerState::kRunning);
    LOG(INFO) << "[SuperMimic] Enter tag=" << param_->GetTag() << ", frames=" << reference_.frames()
              << ", hold_seconds=" << param_->hold_seconds << ", input=1520, actions=25, bridge="
              << (transition_.IsActive() ? "active" : "disabled");
    if (!transition_.IsActive()) LOG(INFO) << "[SuperMimic] Starting policy at frame 0";
    return true;
  } catch (const std::exception& error) {
    LOG(ERROR) << "[SuperMimic] Enter failed: " << error.what();
    policy_.reset();
    SetRunnerState(RunnerState::kFault);
    return false;
  }
}

void RlSupermimicRunner::Run() {
  try {
    if (GetRunnerState() != RunnerState::kRunning) { SendCommand(); return; }
    Eigen::Matrix3d rotation;
    Eigen::Vector3d angular_velocity;
    ReadState(&rotation, &angular_velocity);
    const int frame = sequence_.frame();
    const auto observation = supermimic::BuildObservation(
        actual_q_(mapping_), actual_qd_(mapping_), rotation, angular_velocity,
        yaw_alignment_ * reference_.rotations.at(frame), param_->default_joint_pos, previous_action_);
    supermimic::PushObservation(observation, first_observation_, &history_);
    first_observation_ = false;
    const auto input = supermimic::BuildInput(history_, reference_.goals.row(frame).transpose());
    Eigen::VectorXd action;
    if (!policy_ || !policy_->Infer(input, &action)) throw std::runtime_error("Policy inference failed");
    previous_action_ = action;
    live_command_.q(mapping_) = supermimic::ActionTarget(param_->default_joint_pos, param_->action_scale, action);
    const bool bridging = transition_.IsActive();
    if (!transition_.Apply(live_command_, live_command_.q, actual_q_, param_->control_dt, &command_))
      throw std::runtime_error("Entry bridge failed");
    SendCommand();
    if (bridging) {
      if (!transition_.IsActive()) {
        sequence_.Reset();
        previous_action_.setZero();
        first_observation_ = true;
        yaw_alignment_ = supermimic::Yaw(rotation) * supermimic::Yaw(reference_.rotations.front()).transpose();
        LOG(INFO) << "[SuperMimic] Entry bridge completed; starting policy at frame 0";
      }
    } else {
      sequence_.Advance();
      if (sequence_.finished()) {
        LOG(INFO) << "[SuperMimic] Reference and hold completed; requesting configured auto transition";
        SetRunnerState(RunnerState::kTryExit);
      }
    }
  } catch (const std::exception& error) { Fail(error.what()); }
}

void RlSupermimicRunner::Fail(const std::string& reason) {
  LOG(ERROR) << "[SuperMimic] " << reason << "; requesting passive fallback";
  if (actual_q_.size() == command_.q.size() && actual_q_.allFinite()) command_.q = actual_q_;
  command_.qd.setZero();
  command_.kp.setZero();
  command_.kd.setConstant(8.0);
  command_.tau_ff.setZero();
  SetRunnerState(RunnerState::kFault);
  SendCommand();
}

void RlSupermimicRunner::SendCommand() {
  GetMutableOutput().SetCommand(command_.q, command_.qd, command_.kp, command_.kd, command_.tau_ff);
}

}  // namespace runner

#include "rl_walking_leolab_example/rl_walking_leolab_example_runner.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <unordered_set>
#include <utility>

#include <glog/logging.h>

#include "math/rotation_matrix.h"

namespace runner {
namespace {

Eigen::VectorXd SelectByIndex(const Eigen::VectorXd& values, const Eigen::VectorXi& indices) {
  Eigen::VectorXd selected(indices.size());
  for (int i = 0; i < indices.size(); ++i) {
    selected(i) = values(indices(i));
  }
  return selected;
}

}  // namespace

RlWalkingLeolabExampleRunner::RlWalkingLeolabExampleRunner(
    std::string_view name, const std::shared_ptr<data::DataStore>& data_store)
    : MotionRunner(name, data_store) {
  param_ = data::ParamManager::create<data::RlWalkingLeolabExampleParam>();
}

void RlWalkingLeolabExampleRunner::SetupContext() { data_store_->parallel_by_classic_parser.store(false); }

void RlWalkingLeolabExampleRunner::TeardownContext() {}

bool RlWalkingLeolabExampleRunner::Enter() {
  if (!param_tag_.empty() && param_tag_ != last_param_tag_) {
    param_ = data::ParamManager::create<data::RlWalkingLeolabExampleParam>(param_tag_);
    last_param_tag_ = param_tag_;
  }
  if (!param_ || !ValidateParam()) {
    return false;
  }

  const std::string policy_path =
      common::PathJoin(common::GlobalPathManager::GetInstance().GetConfigPath(), param_->policy_file);
  mlp_net_.reset();
  recurrent_net_.reset();
  if (param_->policy_recurrent) {
    recurrent_net_ = std::make_unique<math::MNNRecurrentModel>(
        policy_path, param_->observation_input_name, param_->hidden_input_name, param_->cell_input_name,
        param_->action_output_name, param_->hidden_output_name, param_->cell_output_name);
    if (!recurrent_net_->IsValid()) {
      LOG(ERROR) << "[RlWalkingLeolabExampleRunner::Enter] Failed to load recurrent policy: " << policy_path;
      return false;
    }
  } else {
    mlp_net_ = std::make_unique<math::MNNModel>(policy_path);
    if (!mlp_net_) {
      LOG(ERROR) << "[RlWalkingLeolabExampleRunner::Enter] Failed to load policy: " << policy_path;
      return false;
    }
  }

  if (!BuildJointMapping() || !BuildOverrideActionIndices()) {
    return false;
  }

  observation_history_.setZero(param_->num_one_step_observations, param_->num_include_obs_steps);
  policy_observation_.setZero(param_->num_observations);
  mlp_net_action_.setZero(param_->num_actions);
  recurrent_hidden_.resize(0);
  recurrent_cell_.resize(0);
  if (param_->policy_recurrent) {
    recurrent_hidden_.setZero(recurrent_net_->HiddenSize());
    recurrent_cell_.setZero(recurrent_net_->CellSize());
  }

  default_joint_q_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  joint_kp_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  joint_kd_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  q_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  qd_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  tau_ff_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);

  default_joint_q_(policy2deploy_joint_idx_) = param_->default_joint_pos;
  joint_kp_(policy2deploy_joint_idx_) = param_->joint_stiffness;
  joint_kd_(policy2deploy_joint_idx_) = param_->joint_damping;
  joint_kp_cmd_ = joint_kp_;
  joint_kd_cmd_ = joint_kd_;
  action_scale_ = param_->action_scale;

  imu_install_bias_ = param_->imu_install_bias.value_or(Eigen::Vector3d::Zero());
  remote_command_shaper_.Configure({
      .speed_pos = param_->command_scale_pos,
      .speed_neg = param_->command_scale_neg,
      .activation_threshold = param_->remote_command_activation_threshold,
      .release_threshold = param_->remote_command_release_threshold,
      .translation_axis_switch_margin = param_->remote_command_translation_axis_switch_margin,
      .reversal_pause_sec = param_->remote_command_reversal_pause_sec,
      .control_dt = param_->control_dt,
  });
  lpf_command_.reset();
  if (param_->enable_remote_command_lpf) {
    lpf_command_ = std::make_unique<math::FirstOrderLowPassFilter<Eigen::Vector3d>>(
        param_->remote_command_sampling_frequency, param_->remote_command_cut_off_frequency);
    lpf_command_->Reset();
  }

  time_ = 0.0;
  is_first_time_ = true;
  policy_output_valid_ = true;
  command_.setZero();
  GetMutableOutput().Reset();

  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_real_);
  data_store_->joint_info.GetState(data::JointInfoType::kVelocity, qd_real_);
  q_des_ = q_real_;
  if (!ValidatePolicyContract()) {
    return false;
  }
  return InitializeEntryTransition();
}

bool RlWalkingLeolabExampleRunner::ValidateParam() const {
  const int expected_one_step = 9 + 3 * param_->num_actions;
  if (param_->num_one_step_observations != expected_one_step) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] one-step observation mismatch: "
               << param_->num_one_step_observations << " != " << expected_one_step;
    return false;
  }
  if (param_->num_observations != param_->num_one_step_observations * param_->num_include_obs_steps) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] num_observations must equal one_step * history";
    return false;
  }
  if (param_->num_include_obs_steps <= 0 || param_->num_actions <= 0) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] num_actions and num_include_obs_steps must be positive";
    return false;
  }
  if (param_->policy_recurrent && param_->num_include_obs_steps != 1) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] recurrent policies require num_include_obs_steps == 1";
    return false;
  }
  if (param_->host_joint_names.size() != static_cast<std::size_t>(param_->num_actions) ||
      param_->joint_names.size() != static_cast<std::size_t>(param_->num_actions) ||
      param_->default_joint_pos.size() != param_->num_actions ||
      param_->joint_stiffness.size() != param_->num_actions ||
      param_->joint_damping.size() != param_->num_actions ||
      param_->action_scale.size() != param_->num_actions) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] Joint/action parameter vector length mismatch";
    return false;
  }
  if (param_->policy_file.empty()) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] policy_file must not be empty";
    return false;
  }
  if (!param_->default_joint_pos.allFinite() || !param_->joint_stiffness.allFinite() ||
      !param_->joint_damping.allFinite() || !param_->action_scale.allFinite()) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] Joint/action parameters contain non-finite values";
    return false;
  }
  if ((param_->joint_stiffness.array() <= 0.0).any() || (param_->joint_damping.array() <= 0.0).any() ||
      (param_->action_scale.array() <= 0.0).any()) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] joint_stiffness, joint_damping, and action_scale must be > 0";
    return false;
  }
  if (!std::isfinite(param_->action_clip) || param_->action_clip <= 0.0 ||
      !std::isfinite(param_->observation_clip) || param_->observation_clip <= 0.0 ||
      !std::isfinite(param_->control_dt) || param_->control_dt <= 0.0) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] clip values and control_dt must be finite and > 0";
    return false;
  }
  if (param_->enable_remote_command_lpf &&
      (!std::isfinite(param_->remote_command_sampling_frequency) ||
       !std::isfinite(param_->remote_command_cut_off_frequency) ||
       param_->remote_command_sampling_frequency <= 0.0 || param_->remote_command_cut_off_frequency <= 0.0)) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] remote command filter parameters must be finite and > 0";
    return false;
  }
  if (!std::isfinite(param_->remote_command_activation_threshold) ||
      !std::isfinite(param_->remote_command_release_threshold) ||
      param_->remote_command_activation_threshold <= 0.0 || param_->remote_command_activation_threshold > 1.0 ||
      param_->remote_command_release_threshold < 0.0 ||
      param_->remote_command_release_threshold >= param_->remote_command_activation_threshold) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] command thresholds require 0 <= release < activation <= 1";
    return false;
  }
  if (!std::isfinite(param_->remote_command_translation_axis_switch_margin) ||
      param_->remote_command_translation_axis_switch_margin < 0.0 ||
      param_->remote_command_translation_axis_switch_margin > 1.0 ||
      !std::isfinite(param_->remote_command_reversal_pause_sec) ||
      param_->remote_command_reversal_pause_sec < 0.0) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] command switch margin and reversal pause are invalid";
    return false;
  }
  if (!param_->command_scale_pos.allFinite() || !param_->command_scale_neg.allFinite() ||
      (param_->command_scale_pos.array() <= 0.0).any() || (param_->command_scale_neg.array() <= 0.0).any()) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] fixed command speeds must be finite and > 0";
    return false;
  }
  if (!std::isfinite(param_->remote_command_tactical_front_offset_deg)) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] tactical-front offset must be finite";
    return false;
  }
  if (param_->imu_install_bias.has_value() && !param_->imu_install_bias->allFinite()) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] imu_install_bias contains non-finite values";
    return false;
  }
  return true;
}

bool RlWalkingLeolabExampleRunner::ValidatePolicyContract() {
  if (param_->policy_recurrent) {
    if (!recurrent_net_ || recurrent_net_->ObservationSize() != param_->num_observations ||
        recurrent_net_->ActionSize() != param_->num_actions ||
        recurrent_net_->HiddenSize() <= 0 || recurrent_net_->CellSize() <= 0) {
      LOG(ERROR) << "[RlWalkingLeolabExampleRunner] Recurrent policy tensor dimensions do not match config";
      return false;
    }
    Eigen::VectorXf action;
    Eigen::VectorXf hidden_out;
    Eigen::VectorXf cell_out;
    const Eigen::VectorXf observation = Eigen::VectorXf::Zero(param_->num_observations);
    const Eigen::VectorXf hidden_in = Eigen::VectorXf::Zero(recurrent_net_->HiddenSize());
    const Eigen::VectorXf cell_in = Eigen::VectorXf::Zero(recurrent_net_->CellSize());
    if (!recurrent_net_->Inference(observation, hidden_in, cell_in, &action, &hidden_out, &cell_out) ||
        action.size() != param_->num_actions || !action.allFinite()) {
      LOG(ERROR) << "[RlWalkingLeolabExampleRunner] Recurrent policy dry-run failed";
      return false;
    }
    return true;
  }

  if (!mlp_net_) return false;
  const Eigen::VectorXd policy_action = (mlp_net_->Inference(policy_observation_.cast<float>())).cast<double>();
  if (policy_action.size() != param_->num_actions) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] Policy output dimension mismatch during Enter: "
               << policy_action.size() << " != " << param_->num_actions;
    return false;
  }
  if (!policy_action.allFinite()) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] Policy dry-run produced non-finite output";
    return false;
  }
  return true;
}

bool RlWalkingLeolabExampleRunner::BuildJointMapping() {
  policy2deploy_joint_idx_.setZero(param_->num_actions);
  std::unordered_set<std::string> host_joint_names;
  std::unordered_set<std::string> deploy_joint_names;
  std::unordered_set<int> deploy_joint_indices;
  for (int i = 0; i < param_->num_actions; ++i) {
    if (!host_joint_names.insert(param_->host_joint_names[i]).second) {
      LOG(ERROR) << "[RlWalkingLeolabExampleRunner] Duplicate host joint at action " << i << ": "
                 << param_->host_joint_names[i];
      return false;
    }
    if (!deploy_joint_names.insert(param_->joint_names[i]).second) {
      LOG(ERROR) << "[RlWalkingLeolabExampleRunner] Duplicate deploy joint at action " << i << ": "
                 << param_->joint_names[i];
      return false;
    }
    const auto it = model_param_->joint_id_in_total_limb.find(param_->joint_names[i]);
    if (it == model_param_->joint_id_in_total_limb.end()) {
      LOG(ERROR) << "[RlWalkingLeolabExampleRunner] Unknown deploy joint: " << param_->joint_names[i]
                 << " mapped from host joint " << param_->host_joint_names[i];
      return false;
    }
    if (!deploy_joint_indices.insert(it->second).second) {
      LOG(ERROR) << "[RlWalkingLeolabExampleRunner] Duplicate deploy joint index at action " << i << ": "
                 << it->second;
      return false;
    }
    policy2deploy_joint_idx_(i) = it->second;
  }
  return true;
}

bool RlWalkingLeolabExampleRunner::BuildOverrideActionIndices() {
  override_action_idx_.resize(static_cast<int>(param_->override_action_joint_names.size()));
  for (int i = 0; i < static_cast<int>(param_->override_action_joint_names.size()); ++i) {
    const auto& joint_name = param_->override_action_joint_names[static_cast<std::size_t>(i)];
    bool found = false;
    for (int action_idx = 0; action_idx < param_->num_actions; ++action_idx) {
      if (param_->host_joint_names[static_cast<std::size_t>(action_idx)] == joint_name) {
        override_action_idx_(i) = action_idx;
        found = true;
        break;
      }
    }
    if (!found) {
      LOG(ERROR) << "[RlWalkingLeolabExampleRunner] override_action_joint_names entry is not a host joint: "
                 << joint_name;
      return false;
    }
  }
  return true;
}

bool RlWalkingLeolabExampleRunner::ComputeBaseState(Eigen::Vector3d* base_ang_vel,
                                                    Eigen::Vector3d* projected_gravity) const {
  if (!base_ang_vel || !projected_gravity) return false;
  const auto imu = data_store_->imu_info.Get();
  if (!imu) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] IMU state unavailable";
    return false;
  }
  const Eigen::Matrix3d r_install = math::RotationMatrixd(math::RollPitchYawd(imu_install_bias_)).matrix();
  const Eigen::Matrix3d r_local = math::RotationMatrixd(imu->quaternion).matrix();
  const Eigen::Matrix3d r_real = r_local * r_install.transpose();
  *base_ang_vel = r_real.transpose() * r_local * imu->angular_velocity;
  *projected_gravity = -r_real.transpose() * Eigen::Vector3d::UnitZ();
  return base_ang_vel->allFinite() && projected_gravity->allFinite();
}

void RlWalkingLeolabExampleRunner::Run() {
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_real_);
  data_store_->joint_info.GetState(data::JointInfoType::kVelocity, qd_real_);

  UpdateRemoteCommand();
  CalculateObservation();
  CalculateMotorCommand();
  ApplyEntryTransition();
  SendMotorCommand();

  time_ += param_->control_dt;
}

TransitionState RlWalkingLeolabExampleRunner::TryExit() {
  const auto joint_override_command = data_store_->joint_override_command.Get();
  if (joint_override_command->IsEnable()) {
    Run();
    return TransitionState::kTrying;
  }
  return TransitionState::kCompleted;
}

bool RlWalkingLeolabExampleRunner::Exit() {
  recurrent_hidden_.setZero();
  recurrent_cell_.setZero();
  mlp_net_action_.setZero();
  return true;
}

void RlWalkingLeolabExampleRunner::End() {}

void RlWalkingLeolabExampleRunner::UpdateRemoteCommand() {
  const auto& gamepad = data_store_->gamepad_info.Get();
  const Eigen::Vector3d raw_command(gamepad->LeftStick_X, gamepad->LeftStick_Y, gamepad->RightStick_Y);
  const Eigen::Vector3d tactical_command = remote_command_shaper_.Update(raw_command);

  // leo_lab samples/teleoperates in a tactical frame, while the policy
  // observation consumes root-yaw-frame velocity. Match its +37 deg rotation.
  const double offset = param_->remote_command_tactical_front_offset_deg * std::acos(-1.0) / 180.0;
  const double cosine = std::cos(offset);
  const double sine = std::sin(offset);
  command_.x() = cosine * tactical_command.x() - sine * tactical_command.y();
  command_.y() = sine * tactical_command.x() + cosine * tactical_command.y();
  command_.z() = tactical_command.z();
  if (param_->enable_remote_command_lpf && lpf_command_) {
    command_ = lpf_command_->Update(command_);
  }
}

void RlWalkingLeolabExampleRunner::CalculateObservation() {
  Eigen::Vector3d base_ang_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d projected_gravity = Eigen::Vector3d::Zero();
  if (!ComputeBaseState(&base_ang_vel, &projected_gravity)) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] Invalid base state; holding current pose";
    policy_output_valid_ = false;
    return;
  }

  const Eigen::VectorXd joint_pos_obs = SelectByIndex(q_real_, policy2deploy_joint_idx_);
  const Eigen::VectorXd joint_vel_obs = SelectByIndex(qd_real_, policy2deploy_joint_idx_);

  Eigen::VectorXd observation_single(param_->num_one_step_observations);
  observation_single << base_ang_vel, projected_gravity, command_, joint_pos_obs, joint_vel_obs, mlp_net_action_;
  observation_single = observation_single.cwiseMax(-param_->observation_clip).cwiseMin(param_->observation_clip);

  if (is_first_time_) {
    is_first_time_ = false;
    observation_history_.colwise() = observation_single;
  } else if (param_->num_include_obs_steps > 1) {
    observation_history_.leftCols(param_->num_include_obs_steps - 1) =
        observation_history_.rightCols(param_->num_include_obs_steps - 1);
    observation_history_.rightCols(1) = observation_single;
  } else {
    observation_history_.rightCols(1) = observation_single;
  }

  policy_observation_.head(param_->num_observations) =
      Eigen::Map<Eigen::VectorXd>(observation_history_.transpose().data(), observation_history_.size());
  policy_output_valid_ = policy_observation_.allFinite();
}

void RlWalkingLeolabExampleRunner::CalculateMotorCommand() {
  if (!policy_output_valid_) {
    HoldCurrentPose();
    return;
  }

  Eigen::VectorXd policy_action;
  if (param_->policy_recurrent) {
    Eigen::VectorXf action;
    Eigen::VectorXf hidden_out;
    Eigen::VectorXf cell_out;
    if (!recurrent_net_ ||
        !recurrent_net_->Inference(policy_observation_.cast<float>(), recurrent_hidden_, recurrent_cell_, &action,
                                   &hidden_out, &cell_out)) {
      LOG(ERROR) << "[RlWalkingLeolabExampleRunner] Recurrent policy inference failed; holding current pose";
      HoldCurrentPose();
      return;
    }
    recurrent_hidden_ = std::move(hidden_out);
    recurrent_cell_ = std::move(cell_out);
    policy_action = action.cast<double>();
  } else {
    if (!mlp_net_) {
      HoldCurrentPose();
      return;
    }
    policy_action = (mlp_net_->Inference(policy_observation_.cast<float>())).cast<double>();
  }
  if (policy_action.size() != param_->num_actions || !policy_action.allFinite()) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] Invalid policy action; holding current pose";
    HoldCurrentPose();
    return;
  }

  mlp_net_action_ = policy_action.cwiseMax(-param_->action_clip).cwiseMin(param_->action_clip);
  Eigen::VectorXd controlled_action = mlp_net_action_;
  for (int i = 0; i < override_action_idx_.size(); ++i) {
    controlled_action(override_action_idx_(i)) = param_->override_action_value;
  }

  q_des_ = default_joint_q_;
  q_des_(policy2deploy_joint_idx_) += controlled_action.cwiseProduct(action_scale_);
  qd_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  tau_ff_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
}

bool RlWalkingLeolabExampleRunner::InitializeEntryTransition() {
  motion_transition::EntryTransitionConfig config;
  config.enabled = param_->entry_transition_enabled.value_or(false);
  config.nominal_duration = param_->entry_transition_duration.value_or(0.16);
  config.min_duration = param_->entry_transition_min_duration.value_or(0.10);
  config.max_duration = param_->entry_transition_max_duration.value_or(0.28);
  config.max_joint_velocity = param_->entry_transition_max_joint_velocity.value_or(8.0);
  config.max_joint_acceleration = param_->entry_transition_max_joint_acceleration.value_or(120.0);
  config.reference_pose_weight = param_->entry_transition_reference_pose_weight.value_or(0.25);
  config.source_command_tracking_error = param_->entry_transition_source_tracking_error.value_or(0.75);
  if (!entry_transition_.Configure(config)) {
    return false;
  }

  motion_transition::JointCommand source;
  motion_transition::CaptureJointCommand(data_store_->joint_info, model_param_->num_total_joints, &source);

  motion_transition::JointCommand fallback;
  fallback.q = default_joint_q_;
  fallback.qd = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  fallback.kp = joint_kp_;
  fallback.kd = joint_kd_;
  fallback.tau_ff = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  entry_reference_q_ = default_joint_q_;
  return entry_transition_.Start(source, q_real_, qd_real_, fallback);
}

void RlWalkingLeolabExampleRunner::ApplyEntryTransition() {
  motion_transition::JointCommand target;
  target.q = q_des_;
  target.qd = qd_des_;
  target.kp = joint_kp_;
  target.kd = joint_kd_;
  target.tau_ff = tau_ff_des_;

  motion_transition::JointCommand command;
  if (!entry_transition_.Apply(target, entry_reference_q_, q_real_, param_->control_dt, &command)) {
    LOG(ERROR) << "[RlWalkingLeolabExampleRunner] Entry transition failed; holding measured pose";
    q_des_ = q_real_;
    qd_des_.setZero();
    tau_ff_des_.setZero();
    joint_kp_cmd_ = joint_kp_;
    joint_kd_cmd_ = joint_kd_;
    return;
  }
  q_des_ = std::move(command.q);
  qd_des_ = std::move(command.qd);
  joint_kp_cmd_ = std::move(command.kp);
  joint_kd_cmd_ = std::move(command.kd);
  tau_ff_des_ = std::move(command.tau_ff);
}

void RlWalkingLeolabExampleRunner::HoldCurrentPose() {
  q_des_ = q_real_;
  qd_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  tau_ff_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  mlp_net_action_.setZero();
}

void RlWalkingLeolabExampleRunner::SendMotorCommand() {
  GetMutableOutput().SetCommand(q_des_, qd_des_, joint_kp_cmd_, joint_kd_cmd_, tau_ff_des_);
}

}  // namespace runner

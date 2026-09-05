#include "rl_getup_example/rl_getup_example_runner.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <unordered_set>
#include <vector>

#include <glog/logging.h>

#include "math/rotation_matrix.h"

namespace {

std::array<double, 3> ToArray3(const Eigen::Vector3d& values) { return {values.x(), values.y(), values.z()}; }

std::vector<double> ToStdVector(const Eigen::VectorXd& values) {
  std::vector<double> result(static_cast<std::size_t>(values.size()));
  for (int i = 0; i < values.size(); ++i) {
    result[static_cast<std::size_t>(i)] = values(i);
  }
  return result;
}

std::vector<double> SelectByIndex(const Eigen::VectorXd& values, const Eigen::VectorXi& indices) {
  std::vector<double> result(static_cast<std::size_t>(indices.size()));
  for (int i = 0; i < indices.size(); ++i) {
    result[static_cast<std::size_t>(i)] = values(indices(i));
  }
  return result;
}

void CopyToEigen(const std::vector<double>& source, Eigen::VectorXd* target) {
  target->resize(static_cast<int>(source.size()));
  for (int i = 0; i < target->size(); ++i) {
    (*target)(i) = source[static_cast<std::size_t>(i)];
  }
}

}  // namespace

namespace runner {

RlGetupExampleRunner::RlGetupExampleRunner(std::string_view name,
                                           const std::shared_ptr<data::DataStore>& data_store)
    : MotionRunner(name, data_store) {
  param_ = data::ParamManager::create<data::RlGetupExampleParam>();
}

void RlGetupExampleRunner::SetupContext() { data_store_->parallel_by_classic_parser.store(false); }

void RlGetupExampleRunner::TeardownContext() {}

bool RlGetupExampleRunner::Enter() {
  if (!param_tag_.empty() && param_tag_ != last_param_tag_) {
    param_ = data::ParamManager::create<data::RlGetupExampleParam>(param_tag_);
    last_param_tag_ = param_tag_;
  }
  if (!param_ || !ValidateParam()) {
    return false;
  }

  const std::string policy_path =
      common::PathJoin(common::GlobalPathManager::GetInstance().GetConfigPath(), param_->policy_file);
  mlp_net_ = std::make_unique<math::MNNModel>(policy_path);
  if (!mlp_net_) {
    LOG(ERROR) << "[RlGetupExampleRunner::Enter] Failed to load policy: " << policy_path;
    return false;
  }

  if (!BuildJointMapping()) {
    return false;
  }

  observation_history_.assign(static_cast<std::size_t>(param_->num_observations), 0.0);
  policy_observation_.setZero(param_->num_observations);
  mlp_net_action_.setZero(param_->num_actions);

  joint_kp_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  joint_kd_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  q_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  qd_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  tau_ff_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);

  joint_kp_(policy2deploy_joint_idx_) = param_->joint_stiffness;
  joint_kd_(policy2deploy_joint_idx_) = param_->joint_damping;
  action_scale_ = param_->action_scale;

  imu_install_bias_ = param_->imu_install_bias.value_or(Eigen::Vector3d::Zero());
  time_ = 0.0;
  is_first_time_ = true;
  success_hold_time_ = 0.0;
  completed_by_posture_ = false;
  timed_out_ = false;
  exit_status_reported_ = false;
  observation_valid_ = true;
  GetMutableOutput().Reset();

  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_real_);
  q_des_ = q_real_;
  if (!ValidatePolicyContract() || !ValidateEntryState()) {
    return false;
  }
  return true;
}

bool RlGetupExampleRunner::ValidateParam() const {
  if (param_->num_one_step_observations != 76 || param_->num_include_obs_steps != 6 ||
      param_->num_observations != 456 || param_->num_actions != 23) {
    LOG(ERROR) << "[RlGetupExampleRunner] HoST ABI mismatch: one_step=" << param_->num_one_step_observations
               << ", history=" << param_->num_include_obs_steps << ", obs=" << param_->num_observations
               << ", actions=" << param_->num_actions;
    return false;
  }
  if (param_->num_observations != param_->num_one_step_observations * param_->num_include_obs_steps) {
    LOG(ERROR) << "[RlGetupExampleRunner] Observation dimension does not equal one_step * history";
    return false;
  }
  if (param_->first_frame_history_mode != "zero_oldest_append_current") {
    LOG(ERROR) << "[RlGetupExampleRunner] Unsupported first_frame_history_mode="
               << param_->first_frame_history_mode;
    return false;
  }
  if (param_->host_joint_names.size() != static_cast<size_t>(param_->num_actions) ||
      param_->joint_names.size() != static_cast<size_t>(param_->num_actions) ||
      param_->default_joint_pos.size() != param_->num_actions ||
      param_->joint_stiffness.size() != param_->num_actions ||
      param_->joint_damping.size() != param_->num_actions ||
      param_->action_scale.size() != param_->num_actions) {
    LOG(ERROR) << "[RlGetupExampleRunner] Joint/action parameter vector length mismatch";
    return false;
  }
  if (param_->policy_file.empty()) {
    LOG(ERROR) << "[RlGetupExampleRunner] policy_file must not be empty";
    return false;
  }
  if (!param_->default_joint_pos.allFinite()) {
    LOG(ERROR) << "[RlGetupExampleRunner] default_joint_pos contains non-finite values";
    return false;
  }
  if (!param_->joint_stiffness.allFinite() || (param_->joint_stiffness.array() <= 0.0).any()) {
    LOG(ERROR) << "[RlGetupExampleRunner] joint_stiffness must be finite and > 0";
    return false;
  }
  if (!param_->joint_damping.allFinite() || (param_->joint_damping.array() <= 0.0).any()) {
    LOG(ERROR) << "[RlGetupExampleRunner] joint_damping must be finite and > 0";
    return false;
  }
  if (!param_->action_scale.allFinite() || (param_->action_scale.array() <= 0.0).any()) {
    LOG(ERROR) << "[RlGetupExampleRunner] action_scale must be finite and > 0";
    return false;
  }
  if (!std::isfinite(param_->action_rescale) || param_->action_rescale <= 0.0 ||
      !std::isfinite(param_->action_clip) || param_->action_clip <= 0.0) {
    LOG(ERROR) << "[RlGetupExampleRunner] action_rescale and action_clip must be finite and > 0";
    return false;
  }
  if (!std::isfinite(param_->observation_scale_angular_vel) || param_->observation_scale_angular_vel <= 0.0 ||
      !std::isfinite(param_->observation_scale_dof_pos) || param_->observation_scale_dof_pos <= 0.0 ||
      !std::isfinite(param_->observation_scale_dof_vel) || param_->observation_scale_dof_vel <= 0.0 ||
      !std::isfinite(param_->observation_clip) || param_->observation_clip <= 0.0) {
    LOG(ERROR) << "[RlGetupExampleRunner] Observation scales and observation_clip must be finite and > 0";
    return false;
  }
  if (!std::isfinite(param_->control_dt) || param_->control_dt <= 0.0f) {
    LOG(ERROR) << "[RlGetupExampleRunner] control_dt must be finite and > 0";
    return false;
  }
  if (param_->imu_install_bias.has_value() && !param_->imu_install_bias->allFinite()) {
    LOG(ERROR) << "[RlGetupExampleRunner] imu_install_bias contains non-finite values";
    return false;
  }
  if (!std::isfinite(param_->entry_max_upright_projected_gravity_z) ||
      !std::isfinite(param_->entry_max_angular_velocity_norm) ||
      !std::isfinite(param_->success_upright_projected_gravity_z) ||
      !std::isfinite(param_->success_max_angular_velocity_norm) ||
      !std::isfinite(param_->success_hold_duration) || !std::isfinite(param_->timeout_duration) ||
      param_->entry_max_angular_velocity_norm <= 0.0 || param_->success_max_angular_velocity_norm <= 0.0 ||
      param_->success_hold_duration <= 0.0 || param_->timeout_duration <= 0.0) {
    LOG(ERROR) << "[RlGetupExampleRunner] Entry/exit safety parameters must be finite and positive";
    return false;
  }
  return true;
}

bool RlGetupExampleRunner::ValidatePolicyContract() {
  const Eigen::VectorXd policy_action = (mlp_net_->Inference(policy_observation_.cast<float>())).cast<double>();
  if (policy_action.size() != param_->num_actions) {
    LOG(ERROR) << "[RlGetupExampleRunner] Policy output dimension mismatch during Enter: " << policy_action.size()
               << " != " << param_->num_actions;
    return false;
  }
  if (!policy_action.allFinite()) {
    LOG(ERROR) << "[RlGetupExampleRunner] Policy dry-run produced non-finite output during Enter";
    return false;
  }
  return true;
}

bool RlGetupExampleRunner::BuildJointMapping() {
  policy2deploy_joint_idx_.setZero(param_->num_actions);
  std::unordered_set<std::string> host_joint_names;
  std::unordered_set<std::string> deploy_joint_names;
  std::unordered_set<int> deploy_joint_indices;
  for (int i = 0; i < param_->num_actions; ++i) {
    if (!host_joint_names.insert(param_->host_joint_names[i]).second) {
      LOG(ERROR) << "[RlGetupExampleRunner] Duplicate HoST joint at action " << i << ": "
                 << param_->host_joint_names[i];
      return false;
    }
    if (!deploy_joint_names.insert(param_->joint_names[i]).second) {
      LOG(ERROR) << "[RlGetupExampleRunner] Duplicate EngineAI joint at action " << i << ": "
                 << param_->joint_names[i];
      return false;
    }
    const auto it = model_param_->joint_id_in_total_limb.find(param_->joint_names[i]);
    if (it == model_param_->joint_id_in_total_limb.end()) {
      LOG(ERROR) << "[RlGetupExampleRunner] Unknown EngineAI joint: " << param_->joint_names[i]
                 << " mapped from HoST joint " << param_->host_joint_names[i];
      return false;
    }
    if (!deploy_joint_indices.insert(it->second).second) {
      LOG(ERROR) << "[RlGetupExampleRunner] Duplicate deploy joint index at action " << i << ": " << it->second;
      return false;
    }
    policy2deploy_joint_idx_(i) = it->second;
  }
  return true;
}

bool RlGetupExampleRunner::ComputeBaseState(Eigen::Vector3d* base_ang_vel, Eigen::Vector3d* projected_gravity) const {
  if (!base_ang_vel || !projected_gravity) return false;
  const auto imu = data_store_->imu_info.Get();
  if (!imu) {
    LOG(ERROR) << "[RlGetupExampleRunner] IMU state unavailable";
    return false;
  }
  const Eigen::Matrix3d R_install = math::RotationMatrixd(math::RollPitchYawd(imu_install_bias_)).matrix();
  const Eigen::Matrix3d R_local = math::RotationMatrixd(imu->quaternion).matrix();
  const Eigen::Matrix3d R_real = R_local * R_install.transpose();
  *base_ang_vel = R_real.transpose() * R_local * imu->angular_velocity;
  *projected_gravity = -R_real.transpose() * Eigen::Vector3d::UnitZ();
  return base_ang_vel->allFinite() && projected_gravity->allFinite();
}

bool RlGetupExampleRunner::ValidateEntryState() {
  Eigen::Vector3d base_ang_vel;
  Eigen::Vector3d projected_gravity;
  if (!ComputeBaseState(&base_ang_vel, &projected_gravity)) {
    LOG(ERROR) << "[RlGetupExampleRunner] Failed to evaluate entry posture";
    return false;
  }
  last_projected_gravity_ = projected_gravity;
  last_base_ang_vel_norm_ = base_ang_vel.norm();
  if (projected_gravity.z() <= param_->entry_max_upright_projected_gravity_z) {
    LOG(ERROR) << "[RlGetupExampleRunner] Rejecting getup entry because robot is already too upright: gravity_z="
               << projected_gravity.z() << " threshold=" << param_->entry_max_upright_projected_gravity_z;
    return false;
  }
  if (last_base_ang_vel_norm_ > param_->entry_max_angular_velocity_norm) {
    LOG(ERROR) << "[RlGetupExampleRunner] Rejecting getup entry because angular velocity norm is too high: "
               << last_base_ang_vel_norm_ << " threshold=" << param_->entry_max_angular_velocity_norm;
    return false;
  }
  return true;
}

void RlGetupExampleRunner::Run() {
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_real_);
  data_store_->joint_info.GetState(data::JointInfoType::kVelocity, qd_real_);

  // Once the safety timeout expires, stop advancing the policy. Keep the last
  // finite command until the task manager performs its fault recovery. A
  // successful getup exits through the task graph's walk_leo transition.
  if (timed_out_) {
    SendMotorCommand();
    return;
  }

  CalculateObservation();
  CalculateMotorCommand();
  SendMotorCommand();

  time_ += param_->control_dt;
  UpdateCompletionState();
}

void RlGetupExampleRunner::CalculateObservation() {
  observation_valid_ = false;
  Eigen::Vector3d base_ang_vel;
  Eigen::Vector3d projected_gravity;
  if (!ComputeBaseState(&base_ang_vel, &projected_gravity)) {
    LOG(ERROR) << "[RlGetupExampleRunner] Failed to compute base state; holding current joints";
    mlp_net_action_.setZero(param_->num_actions);
    q_des_ = q_real_;
    return;
  }
  observation_valid_ = true;
  last_projected_gravity_ = projected_gravity;
  last_base_ang_vel_norm_ = base_ang_vel.norm();

  const std::vector<double> current_obs = rl_getup_contract::BuildStepObservation(
      ToArray3(base_ang_vel), ToArray3(projected_gravity), SelectByIndex(q_real_, policy2deploy_joint_idx_),
      SelectByIndex(qd_real_, policy2deploy_joint_idx_), ToStdVector(mlp_net_action_), param_->action_rescale,
      {.angular_velocity = param_->observation_scale_angular_vel,
       .dof_position = param_->observation_scale_dof_pos,
       .dof_velocity = param_->observation_scale_dof_vel});
  rl_getup_contract::UpdateObservationHistory(observation_history_, param_->num_one_step_observations,
                                              param_->num_include_obs_steps, current_obs,
                                              param_->observation_clip, !is_first_time_);
  CopyToEigen(observation_history_, &policy_observation_);
  is_first_time_ = false;
}

void RlGetupExampleRunner::CalculateMotorCommand() {
  if (!observation_valid_) {
    mlp_net_action_.setZero(param_->num_actions);
    q_des_ = q_real_;
    return;
  }
  const Eigen::VectorXd policy_action = (mlp_net_->Inference(policy_observation_.cast<float>())).cast<double>();
  if (policy_action.size() != param_->num_actions) {
    LOG(ERROR) << "[RlGetupExampleRunner] Policy action size mismatch: " << policy_action.size() << " != "
               << param_->num_actions << "; holding current joints";
    mlp_net_action_.setZero(param_->num_actions);
    q_des_ = q_real_;
    return;
  }

  mlp_net_action_ = policy_action;
  if (!mlp_net_action_.allFinite()) {
    LOG(ERROR) << "[RlGetupExampleRunner] Non-finite policy action; replacing action with zeros";
    mlp_net_action_.setZero(param_->num_actions);
  }
  mlp_net_action_ = mlp_net_action_.cwiseMax(-param_->action_clip).cwiseMin(param_->action_clip);

  q_des_ = q_real_;
  const std::vector<double> joint_targets = rl_getup_contract::ComputeRelativeJointTargets(
      SelectByIndex(q_real_, policy2deploy_joint_idx_), ToStdVector(mlp_net_action_), ToStdVector(action_scale_),
      param_->action_rescale);
  for (int i = 0; i < param_->num_actions; ++i) {
    q_des_(policy2deploy_joint_idx_(i)) = joint_targets[static_cast<std::size_t>(i)];
  }
  ClampAndCheckTargets();
}

void RlGetupExampleRunner::UpdateCompletionState() {
  const bool was_completed = completed_by_posture_;
  if (!observation_valid_) {
    success_hold_time_ = 0.0;
    completed_by_posture_ = false;
  } else {
    const bool upright_and_stable =
        last_projected_gravity_.z() <= param_->success_upright_projected_gravity_z &&
        last_base_ang_vel_norm_ <= param_->success_max_angular_velocity_norm;
    success_hold_time_ = upright_and_stable ? success_hold_time_ + param_->control_dt : 0.0;
    completed_by_posture_ = success_hold_time_ >= param_->success_hold_duration;
  }
  if (completed_by_posture_) {
    // The getup task's normal automatic transition is walk_leo. Keep this
    // separate from the timeout fault path so success and timeout have
    // different automatic destinations.
    timed_out_ = false;
    if (!was_completed) {
      SetRunnerState(RunnerState::kTryExit);
      LOG(INFO) << "[RlGetupExampleRunner] Completion criteria met after " << time_
                << " s; requesting automatic walk_leo transition. gravity_z="
                << last_projected_gravity_.z() << ", ang_vel_norm=" << last_base_ang_vel_norm_;
    }
    return;
  }

  const bool was_timed_out = timed_out_;
  timed_out_ = time_ >= param_->timeout_duration;
  if (timed_out_ && !was_timed_out) {
    // MotionTaskManager routes kFault to idle. The idle task then performs its
    // configured automatic transition to passive.
    SetRunnerState(RunnerState::kFault);
    LOG(WARNING) << "[RlGetupExampleRunner] Timeout after " << time_
                 << " s without proving an upright, stable posture; requesting fault recovery to passive.";
  }
}

void RlGetupExampleRunner::ClampAndCheckTargets() {
  if (param_->clamp_joint_targets) {
    Eigen::VectorXd upper_limit(model_param_->num_total_joints);
    Eigen::VectorXd lower_limit(model_param_->num_total_joints);
    data_store_->joint_info.GetUpperPositionLimit(upper_limit);
    data_store_->joint_info.GetLowerPositionLimit(lower_limit);
    q_des_(policy2deploy_joint_idx_) =
        q_des_(policy2deploy_joint_idx_).cwiseMax(lower_limit(policy2deploy_joint_idx_))
            .cwiseMin(upper_limit(policy2deploy_joint_idx_));
  }

  if (!q_des_(policy2deploy_joint_idx_).allFinite()) {
    LOG(ERROR) << "[RlGetupExampleRunner] Non-finite q_des after target computation; holding current joints";
    q_des_(policy2deploy_joint_idx_) = q_real_(policy2deploy_joint_idx_);
  }
}

void RlGetupExampleRunner::HoldCurrentPose() {
  q_des_ = q_real_;
  qd_des_.setZero(model_param_->num_total_joints);
  tau_ff_des_.setZero(model_param_->num_total_joints);
  GetMutableOutput().SetCommand(q_des_, qd_des_, joint_kp_, joint_kd_, tau_ff_des_);
}

void RlGetupExampleRunner::SendMotorCommand() {
  qd_des_.setZero(model_param_->num_total_joints);
  tau_ff_des_.setZero(model_param_->num_total_joints);
  GetMutableOutput().SetCommand(q_des_, qd_des_, joint_kp_, joint_kd_, tau_ff_des_);
}

TransitionState RlGetupExampleRunner::TryExit() {
  if (timed_out_) {
    if (!exit_status_reported_) {
      LOG(WARNING) << "[RlGetupExampleRunner] Leaving a timed-out getup; fault recovery will route through idle to "
                   << "passive";
      exit_status_reported_ = true;
    }
  } else if (completed_by_posture_) {
    if (!exit_status_reported_) {
      LOG(INFO) << "[RlGetupExampleRunner] Completion criteria met after " << time_ << " s; gravity_z="
                << last_projected_gravity_.z() << ", ang_vel_norm=" << last_base_ang_vel_norm_;
      exit_status_reported_ = true;
    }
  } else {
    if (!exit_status_reported_) {
      LOG(INFO) << "[RlGetupExampleRunner] Operator requested a manual transition before getup completion";
      exit_status_reported_ = true;
    }
  }

  // Manual transitions are allowed to hand off immediately. Do not continue
  // policy inference during the handoff cycle.
  HoldCurrentPose();
  return TransitionState::kCompleted;
}

bool RlGetupExampleRunner::IsTransitionAllowed(std::string_view target_motion) const {
  // Manual recovery and all four manual gait selections are always
  // available. Automatic success still uses the task graph's walk_leo
  // transition, while timeout is handled separately through kFault.
  const bool is_recovery = target_motion == "passive" || target_motion == "pd_stand";
  const bool is_allowed_walk = target_motion == "walk" || target_motion == "walk_custom" ||
                               target_motion == "walk_leo" || target_motion == "walk_leo_terrain";
  return is_recovery || is_allowed_walk;
}

bool RlGetupExampleRunner::Exit() { return true; }

void RlGetupExampleRunner::End() {}

}  // namespace runner

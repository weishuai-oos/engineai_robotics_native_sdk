#include "pd_stand/pd_stand_runner.h"

#include <cmath>
#include <exception>

#include <glog/logging.h>

#include "pd_stand/pd_pose_interpolation.h"
#include "tool/concatenate_vector.h"

namespace runner {

void PdStandRunner::SetupContext() { data_store_->parallel_by_classic_parser.store(true); }

void PdStandRunner::TeardownContext() {}

bool PdStandRunner::Enter() {
  if (!param_tag_.empty()) {
    param_ = data::ParamManager::create<data::PdStandParam>(param_tag_);
  }
  if (!param_ || !LoadCommandParameters()) {
    LOG(ERROR) << "[PdStandRunner] invalid parameters for tag " << param_tag_;
    return false;
  }

  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_init_);
  if (q_init_.size() != q_des_.size() || !q_init_.allFinite()) {
    LOG(ERROR) << "[PdStandRunner] invalid measured joint position size=" << q_init_.size()
               << ", expected=" << q_des_.size();
    return false;
  }

  auto_transition_ = param_->auto_transition.value_or(false);
  iter_ = 0;
  motion_complete_ = false;
  tau_ff_cmd_ = Eigen::VectorXd::Zero(q_init_.size());

  if (!common::IsInMujoco() && !CheckJointPositionBias()) {
    const auto gamepad_info = data_store_->gamepad_info.Get();
    constexpr double kForceStartTrigger = 0.8;
    const bool force_start = global_options_param_ && !global_options_param_->strict_motion_check &&
                             gamepad_info && gamepad_info->LT > kForceStartTrigger;
    if (!force_start) return false;
    LOG(WARNING) << "[PdStandRunner] using force-start override for tag " << param_->GetTag();
  }

  q_cmd_ = q_init_;
  qd_cmd_ = Eigen::VectorXd::Zero(q_init_.size());
  GetMutableOutput().SetCommand(q_cmd_, qd_cmd_, kp_, kd_, tau_ff_cmd_);
  return true;
}

bool PdStandRunner::LoadCommandParameters() {
  try {
    q_des_ = common::ConcatenateVectors(param_->desired_joint_position);
    kp_ = common::ConcatenateVectors(param_->stiffness);
    kd_ = common::ConcatenateVectors(param_->damping);
  } catch (const std::exception& error) {
    LOG(ERROR) << "[PdStandRunner] failed to flatten command parameters: " << error.what();
    return false;
  }

  const int expected = model_param_ ? model_param_->num_total_joints : 0;
  const bool valid_vectors = expected > 0 && q_des_.size() == expected && kp_.size() == expected &&
                             kd_.size() == expected && q_des_.allFinite() && kp_.allFinite() && kd_.allFinite();
  if (!valid_vectors || !std::isfinite(param_->duration) || param_->duration <= 0.0 ||
      !std::isfinite(runner_period_) || runner_period_ <= 0.0) {
    return false;
  }
  duration_ = param_->duration;
  return true;
}

bool PdStandRunner::CheckJointPositionBias() {
  constexpr double kDefaultBiasThreshold = 1.0;
  const double threshold = param_->initial_joint_position_bias_threshold.value_or(kDefaultBiasThreshold);
  if (!std::isfinite(threshold) || threshold < 0.0) return false;

  const Eigen::VectorXd bias = q_init_ - q_des_;
  const auto large_bias = (bias.array().abs() > threshold).matrix();
  if (!large_bias.any()) return true;

  LOG(WARNING) << "[PdStandRunner] joint position bias exceeds " << threshold << " rad";
  for (int i = 0; i < bias.size(); ++i) {
    if (large_bias(i)) {
      LOG(WARNING) << "joint " << model_param_->GetJointNameByJointId(i) << " bias=" << bias[i];
    }
  }
  return false;
}

void PdStandRunner::Run() {
  data_store_->reference_contact_signal_info->SetAllLegContactSignal();
  data_store_->reference_contact_signal_info->SetAllArmNonContactSignal();

  const double phase = std::min(static_cast<double>(iter_) * runner_period_, duration_);
  if (phase >= duration_) {
    q_cmd_ = q_des_;
    qd_cmd_.setZero(q_des_.size());
    motion_complete_ = true;
  } else {
    pd_stand::InterpolateQuintic(q_init_, q_des_, duration_, phase, q_cmd_, qd_cmd_);
  }
  GetMutableOutput().SetCommand(q_cmd_, qd_cmd_, kp_, kd_, tau_ff_cmd_);
  ++iter_;

  if (motion_complete_ && auto_transition_) {
    LOG(INFO) << "[PdStandRunner] completed " << duration_ << " s for tag " << param_->GetTag();
    SetRunnerState(RunnerState::kTryExit);
  }
}

TransitionState PdStandRunner::TryExit() {
  return TransitionState::kCompleted;
}

bool PdStandRunner::Exit() { return true; }

void PdStandRunner::End() {}

}  // namespace runner

#include "idle/idle_runner.h"

#include <cstddef>
#include <glog/logging.h>
#include <iostream>
#include <sstream>

#include "parameter/global_config_initializer.h"
#include "tool/concatenate_vector.h"
#include "variant_store/variant_store.h"

namespace runner {
void IdleRunner::SetupContext() {}

void IdleRunner::TeardownContext() {}

bool IdleRunner::Enter() {
  num_try_exit_ = 0;
  motor_enable_phase_ = MotorEnablePhase::kSynchronizingDisabled;
  data_store_->enable_or_disable_motor.store(false);
  motor_debug_subscriber_ = data::VariantStore::GetInstance().CreateSubscriber<data::MotorDebug>(
      "hardware/motor_debug");

  if (!param_tag_.empty()) {
    param_ = data::ParamManager::create<data::IdleParam>(param_tag_);
  }

  return true;
}

void IdleRunner::Run() {
  const int n = model_param_->num_total_joints;
  Eigen::VectorXd q_cur = Eigen::VectorXd::Zero(n);
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_cur);

  if (!param_) {
    LOG_FIRST_N(WARNING, 1) << "[IdleRunner] param_ is null, output reset to zero.";
    GetMutableOutput().Reset();
    SetRunnerState(runner::RunnerState::kTryExit);
    return;
  }

  const bool pre_motion_cmd_enable = param_->pre_motion_cmd_enable.value_or(false);

  if (pre_motion_cmd_enable) {
    Eigen::VectorXd q_cmd = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd qd_cmd = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd kp_cmd = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd kd_cmd = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd tau_ff_cmd = Eigen::VectorXd::Zero(n);

    const Eigen::VectorXi enable_mask = param_->pre_cmd_enable_joints
                                            ? common::ConcatenateVectors(param_->pre_cmd_enable_joints.value())
                                            : Eigen::VectorXi::Zero(n);
    const Eigen::VectorXd kp =
        param_->stiffness ? common::ConcatenateVectors(param_->stiffness.value()) : Eigen::VectorXd::Zero(n);
    const Eigen::VectorXd kd =
        param_->damping ? common::ConcatenateVectors(param_->damping.value()) : Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) {
      if (enable_mask[i] == 1) {
        q_cmd[i] = q_cur[i];
        kp_cmd[i] = kp[i];
        kd_cmd[i] = kd[i];
      }
    }

    GetMutableOutput().SetCommand(q_cmd, qd_cmd, kp_cmd, kd_cmd, tau_ff_cmd);
  } else {
    GetMutableOutput().Reset();
  }

  // The task graph declares idle -> passive as its automatic transition. This
  // must also be honored on hardware because getup timeout recovery enters
  // idle through MotionTaskManager's fault path.
  SetRunnerState(runner::RunnerState::kTryExit);
}

TransitionState IdleRunner::TryExit() {
  if (common::IsInMujoco()) {
    return TransitionState::kCompleted;
  }

  std::string reason;
  const MotorFeedbackState feedback = ReadMotorFeedback(&reason);
  if (feedback == MotorFeedbackState::kFault) {
    data_store_->enable_or_disable_motor.store(false);
    if (!RetryOrFault("motor feedback", reason)) return TransitionState::kCompleted;
    this->Run();
    return TransitionState::kTrying;
  }

  if (motor_enable_phase_ == MotorEnablePhase::kSynchronizingDisabled) {
    // Require a fresh disabled acknowledgement before asserting enable. This
    // prevents a stale enabled_motor_state value from being mistaken for a
    // successful re-enable after an external motor disable.
    data_store_->enable_or_disable_motor.store(false);
    if (feedback == MotorFeedbackState::kDisabled) {
      motor_enable_phase_ = MotorEnablePhase::kWaitingForEnabled;
      num_try_exit_ = 0;
      LOG(INFO) << "[IdleRunner] Motor disable state synchronized; requesting enable and waiting for feedback";
    } else if (!RetryOrFault("motor disable synchronization", reason)) {
      return TransitionState::kCompleted;
    }
  }

  if (motor_enable_phase_ == MotorEnablePhase::kWaitingForEnabled) {
    data_store_->enable_or_disable_motor.store(true);
    if (feedback == MotorFeedbackState::kEnabled) {
      LOG(INFO) << "[IdleRunner] Motor enable confirmed by feedback";
      return TransitionState::kCompleted;
    }
    if (!RetryOrFault("motor enable confirmation", reason)) return TransitionState::kCompleted;
  }

  this->Run();
  return TransitionState::kTrying;
}

IdleRunner::MotorFeedbackState IdleRunner::ReadMotorFeedback(std::string* reason) const {
  if (reason) reason->clear();

  const bool enabled_feedback = data_store_->enabled_motor_state.load();
  if (!motor_debug_subscriber_.IsValid()) {
    return enabled_feedback ? MotorFeedbackState::kEnabled : MotorFeedbackState::kDisabled;
  }

  const auto debug = motor_debug_subscriber_.Get();
  if (!debug || debug->Size() == 0) {
    if (reason) *reason = "motor_debug feedback is not available yet";
    return MotorFeedbackState::kUnavailable;
  }

  const std::size_t joint_count = static_cast<std::size_t>(model_param_->num_total_joints);
  if (debug->enable.size() < joint_count || debug->offline.size() < joint_count ||
      debug->error_code.size() < joint_count) {
    if (reason) {
      std::ostringstream stream;
      stream << "motor_debug feedback has incomplete vectors, enable=" << debug->enable.size()
             << ", offline=" << debug->offline.size() << ", error_code=" << debug->error_code.size()
             << ", required=" << joint_count;
      *reason = stream.str();
    }
    return MotorFeedbackState::kFault;
  }

  for (std::size_t i = 0; i < joint_count; ++i) {
    if (debug->offline[i] != 0 || debug->error_code[i] != 0) {
      if (reason) {
        std::ostringstream stream;
        stream << "motor_debug reports joint " << i << " offline="
               << static_cast<int>(debug->offline[i]) << " error_code=" << debug->error_code[i];
        *reason = stream.str();
      }
      return MotorFeedbackState::kFault;
    }
  }

  bool all_enabled = true;
  bool all_disabled = true;
  for (std::size_t i = 0; i < joint_count; ++i) {
    all_enabled = all_enabled && debug->enable[i] != 0;
    all_disabled = all_disabled && debug->enable[i] == 0;
  }

  if (enabled_feedback && all_enabled) return MotorFeedbackState::kEnabled;
  if (!enabled_feedback && all_disabled) return MotorFeedbackState::kDisabled;
  if (reason) *reason = "enabled_motor_state and motor_debug enable vectors disagree";
  return MotorFeedbackState::kUnavailable;
}

bool IdleRunner::RetryOrFault(const char* phase, const std::string& reason) {
  if (num_try_exit_ < kNumTryExit) {
    ++num_try_exit_;
    if (num_try_exit_ == 1 || num_try_exit_ % 100 == 0) {
      LOG(WARNING) << "[IdleRunner] Waiting for " << phase << " attempt " << num_try_exit_ << "/"
                   << kNumTryExit << ": " << (reason.empty() ? "feedback not ready" : reason);
    }
    return true;
  }

  data_store_->enable_or_disable_motor.store(false);
  SetRunnerState(runner::RunnerState::kFault);
  LOG(ERROR) << "[IdleRunner] Motor " << phase << " timed out after " << kNumTryExit
             << " cycles; keeping motor enable request off. " << reason;
  return false;
}

bool IdleRunner::Exit() { return true; }

void IdleRunner::End() {}
}  // namespace runner

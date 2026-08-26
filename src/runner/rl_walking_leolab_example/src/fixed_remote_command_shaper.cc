#include "rl_walking_leolab_example/fixed_remote_command_shaper.h"

#include <algorithm>
#include <cmath>

namespace runner {
namespace {

int Sign(double value) {
  if (value > 0.0) return 1;
  if (value < 0.0) return -1;
  return 0;
}

}  // namespace

void FixedRemoteCommandShaper::Configure(const FixedRemoteCommandShaperConfig& config) {
  config_ = config;
  Reset();
}

void FixedRemoteCommandShaper::Reset() {
  active_translation_axis_ = TranslationAxis::kNone;
  yaw_active_ = false;
  last_nonzero_sign_.setZero();
  zero_elapsed_sec_.setConstant(config_.reversal_pause_sec);
}

Eigen::Vector3d FixedRemoteCommandShaper::Update(const Eigen::Vector3d& raw_command) {
  if (!raw_command.allFinite()) {
    Reset();
    return Eigen::Vector3d::Zero();
  }

  UpdateTranslationAxis(raw_command);
  UpdateYawActive(raw_command.z());

  Eigen::Vector3d desired_command = Eigen::Vector3d::Zero();
  if (active_translation_axis_ == TranslationAxis::kForward) {
    desired_command.x() = raw_command.x() > 0.0 ? config_.speed_pos.x() : -config_.speed_neg.x();
  } else if (active_translation_axis_ == TranslationAxis::kLateral) {
    desired_command.y() = raw_command.y() > 0.0 ? config_.speed_pos.y() : -config_.speed_neg.y();
  }
  if (yaw_active_) {
    desired_command.z() = raw_command.z() > 0.0 ? config_.speed_pos.z() : -config_.speed_neg.z();
  }

  Eigen::Vector3d shaped_command;
  for (int axis = 0; axis < shaped_command.size(); ++axis) {
    shaped_command(axis) = ApplyReversalPause(axis, desired_command(axis));
  }
  return shaped_command;
}

void FixedRemoteCommandShaper::UpdateTranslationAxis(const Eigen::Vector3d& raw_command) {
  const double forward_magnitude = std::abs(raw_command.x());
  const double lateral_magnitude = std::abs(raw_command.y());

  if (active_translation_axis_ == TranslationAxis::kForward) {
    if (forward_magnitude <= config_.release_threshold) {
      active_translation_axis_ = TranslationAxis::kNone;
    } else if (lateral_magnitude >= config_.activation_threshold &&
               lateral_magnitude > forward_magnitude + config_.translation_axis_switch_margin) {
      active_translation_axis_ = TranslationAxis::kLateral;
    }
  } else if (active_translation_axis_ == TranslationAxis::kLateral) {
    if (lateral_magnitude <= config_.release_threshold) {
      active_translation_axis_ = TranslationAxis::kNone;
    } else if (forward_magnitude >= config_.activation_threshold &&
               forward_magnitude > lateral_magnitude + config_.translation_axis_switch_margin) {
      active_translation_axis_ = TranslationAxis::kForward;
    }
  }

  if (active_translation_axis_ != TranslationAxis::kNone) return;
  if (forward_magnitude < config_.activation_threshold && lateral_magnitude < config_.activation_threshold) {
    return;
  }
  active_translation_axis_ = forward_magnitude >= lateral_magnitude ? TranslationAxis::kForward
                                                                    : TranslationAxis::kLateral;
}

void FixedRemoteCommandShaper::UpdateYawActive(double raw_yaw) {
  const double yaw_magnitude = std::abs(raw_yaw);
  if (yaw_active_) {
    if (yaw_magnitude <= config_.release_threshold) yaw_active_ = false;
  } else if (yaw_magnitude >= config_.activation_threshold) {
    yaw_active_ = true;
  }
}

double FixedRemoteCommandShaper::ApplyReversalPause(int axis, double desired_command) {
  const int desired_sign = Sign(desired_command);
  if (desired_sign == 0) {
    zero_elapsed_sec_(axis) =
        std::min(config_.reversal_pause_sec, zero_elapsed_sec_(axis) + config_.control_dt);
    return 0.0;
  }

  if (last_nonzero_sign_(axis) != 0 && desired_sign != last_nonzero_sign_(axis) &&
      zero_elapsed_sec_(axis) < config_.reversal_pause_sec) {
    zero_elapsed_sec_(axis) =
        std::min(config_.reversal_pause_sec, zero_elapsed_sec_(axis) + config_.control_dt);
    return 0.0;
  }

  last_nonzero_sign_(axis) = desired_sign;
  zero_elapsed_sec_(axis) = 0.0;
  return desired_command;
}

}  // namespace runner

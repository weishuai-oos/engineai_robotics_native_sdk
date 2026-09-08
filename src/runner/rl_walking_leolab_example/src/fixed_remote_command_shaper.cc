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

double MoveTowards(double current, double target, double max_delta) {
  const double delta = target - current;
  // Assign the target exactly, including zero at the end of normal braking.
  if (std::abs(delta) <= max_delta + 1e-12) return target;
  return current + std::copysign(max_delta, delta);
}

}  // namespace

bool FixedRemoteCommandShaper::Configure(const FixedRemoteCommandShaperConfig& config) {
  if (config.translation_proportional && !config.translation_slew_enabled) {
    Reset();
    return false;
  }
  config_ = config;
  Reset();
  return true;
}

void FixedRemoteCommandShaper::Reset() {
  active_translation_axis_ = TranslationAxis::kNone;
  translation_activation_candidate_ = TranslationAxis::kNone;
  translation_activation_elapsed_sec_ = 0.0;
  yaw_active_ = false;
  yaw_activation_candidate_sign_ = 0;
  yaw_activation_elapsed_sec_ = 0.0;
  last_nonzero_sign_.setZero();
  zero_elapsed_sec_.setConstant(config_.reversal_pause_sec);
  target_command_.setZero();
  translation_command_.setZero();
}

Eigen::Vector3d FixedRemoteCommandShaper::Update(const Eigen::Vector3d& raw_command, bool input_available) {
  if (!input_available || !raw_command.allFinite()) {
    Reset();
    return Eigen::Vector3d::Zero();
  }

  UpdateTranslationAxis(raw_command);
  UpdateYawActive(raw_command.z());

  target_command_.setZero();
  if (active_translation_axis_ == TranslationAxis::kForward) {
    target_command_.x() = TranslationTarget(0, raw_command.x());
  } else if (active_translation_axis_ == TranslationAxis::kLateral) {
    target_command_.y() = TranslationTarget(1, raw_command.y());
  }
  if (yaw_active_) {
    target_command_.z() = raw_command.z() > 0.0 ? config_.speed_pos.z() : -config_.speed_neg.z();
  }

  Eigen::Vector3d shaped_command;
  if (config_.translation_slew_enabled) {
    shaped_command.head<2>() = ShapeTranslation(target_command_.head<2>());
  } else {
    for (int axis = 0; axis < 2; ++axis) {
      shaped_command(axis) = ApplyReversalPause(axis, target_command_(axis));
    }
  }
  shaped_command.z() = ApplyReversalPause(2, target_command_.z());
  return shaped_command;
}

double FixedRemoteCommandShaper::TranslationTarget(int axis, double raw_command) const {
  const double speed = raw_command > 0.0 ? config_.speed_pos(axis) : -config_.speed_neg(axis);
  if (!config_.translation_proportional) return speed;
  // Remap the active part of the stick. A configured target floor avoids
  // sustained commands below the policy's usable speed; the output ramp still
  // starts at zero, so this floor must be used with translation slew enabled.
  const double magnitude = std::clamp(std::abs(raw_command), 0.0, 1.0);
  if (magnitude <= config_.activation_threshold) return 0.0;
  const double fraction = std::clamp((magnitude - config_.activation_threshold) /
                                        (1.0 - config_.activation_threshold),
                                    0.0, 1.0);
  const double target_speed = config_.translation_min_speed +
                              (std::abs(speed) - config_.translation_min_speed) * fraction;
  return std::copysign(target_speed, speed);
}

Eigen::Vector2d FixedRemoteCommandShaper::ShapeTranslation(Eigen::Vector2d desired_command) {
  // Brake the old direction fully before starting a different axis or sign.
  // In particular, the reversal pause counts actual zero-command time, not
  // the time spent decelerating toward zero.
  for (int axis = 0; axis < 2; ++axis) {
    if (translation_command_(axis) != 0.0 &&
        Sign(desired_command(axis)) != Sign(translation_command_(axis))) {
      desired_command.setZero();
      break;
    }
  }
  for (int axis = 0; axis < 2; ++axis) {
    const double current = translation_command_(axis);
    if (current == 0.0) {
      const double guarded_target = ApplyReversalPause(axis, desired_command(axis));
      translation_command_(axis) =
          MoveTowards(0.0, guarded_target, config_.translation_acceleration * config_.control_dt);
    } else {
      const double rate = std::abs(desired_command(axis)) > std::abs(current)
                              ? config_.translation_acceleration
                              : config_.translation_deceleration;
      translation_command_(axis) = MoveTowards(current, desired_command(axis), rate * config_.control_dt);
      zero_elapsed_sec_(axis) = 0.0;
      if (translation_command_(axis) != 0.0) last_nonzero_sign_(axis) = Sign(translation_command_(axis));
    }
  }
  return translation_command_;
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
      translation_activation_candidate_ = TranslationAxis::kNone;
      translation_activation_elapsed_sec_ = 0.0;
    }
  } else if (active_translation_axis_ == TranslationAxis::kLateral) {
    if (lateral_magnitude <= config_.release_threshold) {
      active_translation_axis_ = TranslationAxis::kNone;
    } else if (forward_magnitude >= config_.activation_threshold &&
               forward_magnitude > lateral_magnitude + config_.translation_axis_switch_margin) {
      active_translation_axis_ = TranslationAxis::kForward;
      translation_activation_candidate_ = TranslationAxis::kNone;
      translation_activation_elapsed_sec_ = 0.0;
    }
  }

  if (active_translation_axis_ != TranslationAxis::kNone) return;
  if (forward_magnitude < config_.activation_threshold && lateral_magnitude < config_.activation_threshold) {
    translation_activation_candidate_ = TranslationAxis::kNone;
    translation_activation_elapsed_sec_ = 0.0;
    return;
  }

  const TranslationAxis candidate =
      forward_magnitude >= lateral_magnitude ? TranslationAxis::kForward : TranslationAxis::kLateral;
  if (candidate != translation_activation_candidate_) {
    translation_activation_candidate_ = candidate;
    translation_activation_elapsed_sec_ = 0.0;
  }
  if (config_.activation_debounce_sec <= 0.0) {
    active_translation_axis_ = candidate;
    translation_activation_candidate_ = TranslationAxis::kNone;
    translation_activation_elapsed_sec_ = 0.0;
    return;
  }

  translation_activation_elapsed_sec_ += config_.control_dt;
  if (translation_activation_elapsed_sec_ >= config_.activation_debounce_sec) {
    active_translation_axis_ = candidate;
    translation_activation_candidate_ = TranslationAxis::kNone;
    translation_activation_elapsed_sec_ = 0.0;
  }
}

void FixedRemoteCommandShaper::UpdateYawActive(double raw_yaw) {
  const double yaw_magnitude = std::abs(raw_yaw);
  if (yaw_active_) {
    if (yaw_magnitude <= config_.release_threshold) {
      yaw_active_ = false;
      yaw_activation_candidate_sign_ = 0;
      yaw_activation_elapsed_sec_ = 0.0;
    }
    return;
  }

  if (yaw_magnitude < config_.activation_threshold) {
    yaw_activation_candidate_sign_ = 0;
    yaw_activation_elapsed_sec_ = 0.0;
    return;
  }

  const int candidate_sign = Sign(raw_yaw);
  if (candidate_sign != yaw_activation_candidate_sign_) {
    yaw_activation_candidate_sign_ = candidate_sign;
    yaw_activation_elapsed_sec_ = 0.0;
  }
  if (config_.activation_debounce_sec <= 0.0) {
    yaw_active_ = true;
    yaw_activation_candidate_sign_ = 0;
    yaw_activation_elapsed_sec_ = 0.0;
    return;
  }

  yaw_activation_elapsed_sec_ += config_.control_dt;
  if (yaw_activation_elapsed_sec_ >= config_.activation_debounce_sec) {
    yaw_active_ = true;
    yaw_activation_candidate_sign_ = 0;
    yaw_activation_elapsed_sec_ = 0.0;
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

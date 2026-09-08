#pragma once

#include <Eigen/Core>

namespace runner {

struct FixedRemoteCommandShaperConfig {
  Eigen::Vector3d speed_pos = Eigen::Vector3d::Ones();
  Eigen::Vector3d speed_neg = Eigen::Vector3d::Ones();
  double activation_threshold = 0.2;
  double release_threshold = 0.12;
  // Require the stick to remain above activation_threshold before issuing a
  // fixed-speed command. Zero preserves the historical one-sample behavior.
  double activation_debounce_sec = 0.0;
  double translation_axis_switch_margin = 0.1;
  double reversal_pause_sec = 0.1;
  double control_dt = 0.02;
  // Opt-in so existing profiles retain their fixed-speed behavior.
  bool translation_proportional = false;
  bool translation_slew_enabled = false;
  double translation_acceleration = 1.0;  // m/s^2
  double translation_deceleration = 2.0;  // m/s^2
  double translation_min_speed = 0.0;     // nonzero proportional target floor, m/s
};

// Shapes tactical-frame sticks. Translation supports proportional speeds and
// finite-time ramps; yaw retains the original fixed-speed/debounce behavior.
// Only one translation axis moves at a time, including during axis changes.
class FixedRemoteCommandShaper {
 public:
  // Returns false (and clears state) for proportional mode without slew.
  bool Configure(const FixedRemoteCommandShaperConfig& config);
  void Reset();
  Eigen::Vector3d Update(const Eigen::Vector3d& raw_command, bool input_available = true);
  const Eigen::Vector3d& TargetCommand() const { return target_command_; }

 private:
  enum class TranslationAxis { kNone, kForward, kLateral };

  void UpdateTranslationAxis(const Eigen::Vector3d& raw_command);
  void UpdateYawActive(double raw_yaw);
  double ApplyReversalPause(int axis, double desired_command);
  double TranslationTarget(int axis, double raw_command) const;
  Eigen::Vector2d ShapeTranslation(Eigen::Vector2d desired_command);

  FixedRemoteCommandShaperConfig config_;
  TranslationAxis active_translation_axis_ = TranslationAxis::kNone;
  TranslationAxis translation_activation_candidate_ = TranslationAxis::kNone;
  double translation_activation_elapsed_sec_ = 0.0;
  bool yaw_active_ = false;
  int yaw_activation_candidate_sign_ = 0;
  double yaw_activation_elapsed_sec_ = 0.0;
  Eigen::Vector3i last_nonzero_sign_ = Eigen::Vector3i::Zero();
  Eigen::Vector3d zero_elapsed_sec_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_command_ = Eigen::Vector3d::Zero();
  Eigen::Vector2d translation_command_ = Eigen::Vector2d::Zero();
};

}  // namespace runner

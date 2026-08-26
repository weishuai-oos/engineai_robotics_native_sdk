#pragma once

#include <Eigen/Core>

namespace runner {

struct FixedRemoteCommandShaperConfig {
  Eigen::Vector3d speed_pos = Eigen::Vector3d::Ones();
  Eigen::Vector3d speed_neg = Eigen::Vector3d::Ones();
  double activation_threshold = 0.2;
  double release_threshold = 0.12;
  double translation_axis_switch_margin = 0.1;
  double reversal_pause_sec = 0.1;
  double control_dt = 0.02;
};

// Converts analog tactical-frame sticks into fixed-speed tactical commands.
// Translation is limited to one dominant axis; yaw remains independent.
class FixedRemoteCommandShaper {
 public:
  void Configure(const FixedRemoteCommandShaperConfig& config);
  void Reset();
  Eigen::Vector3d Update(const Eigen::Vector3d& raw_command);

 private:
  enum class TranslationAxis { kNone, kForward, kLateral };

  void UpdateTranslationAxis(const Eigen::Vector3d& raw_command);
  void UpdateYawActive(double raw_yaw);
  double ApplyReversalPause(int axis, double desired_command);

  FixedRemoteCommandShaperConfig config_;
  TranslationAxis active_translation_axis_ = TranslationAxis::kNone;
  bool yaw_active_ = false;
  Eigen::Vector3i last_nonzero_sign_ = Eigen::Vector3i::Zero();
  Eigen::Vector3d zero_elapsed_sec_ = Eigen::Vector3d::Zero();
};

}  // namespace runner

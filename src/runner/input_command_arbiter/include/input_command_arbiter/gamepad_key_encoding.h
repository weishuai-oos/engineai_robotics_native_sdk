#pragma once

#include <cmath>

namespace runner {

// The pre-built SDK task parser exposes only digital key names. Opposite
// directions on either d-pad axis cannot be pressed together by the hardware,
// so the input adapters use the horizontal pair as a private LT marker and the
// vertical pair as a private RT marker. task_motion maps these pairs back to
// the physical trigger + key combinations.
// Keep these indices aligned with GamepadTool::kKeyString in the SDK core ABI.
inline constexpr int kRtTaskModifierUpIndex = 8;    // CROSS_X_UP
inline constexpr int kRtTaskModifierDownIndex = 9;  // CROSS_X_DOWN
inline constexpr int kLtTaskModifierLeftIndex = 10;   // CROSS_Y_LEFT
inline constexpr int kLtTaskModifierRightIndex = 11;  // CROSS_Y_RIGHT
inline constexpr int kRtTaskModifierKeyValue =
    (1 << kRtTaskModifierUpIndex) | (1 << kRtTaskModifierDownIndex);
inline constexpr int kLtTaskModifierKeyValue =
    (1 << kLtTaskModifierLeftIndex) | (1 << kLtTaskModifierRightIndex);
inline constexpr double kRtTaskModifierThreshold = 0.5;
inline constexpr double kLtTaskModifierThreshold = 0.5;

inline bool IsRtTaskModifierPressed(double value) {
  return std::isfinite(value) && value >= kRtTaskModifierThreshold;
}

inline bool IsLtTaskModifierPressed(double value) {
  return std::isfinite(value) && value >= kLtTaskModifierThreshold;
}

inline void AddRtTaskModifier(double value, int* combined_key_value) {
  if (combined_key_value != nullptr && IsRtTaskModifierPressed(value)) {
    *combined_key_value |= kRtTaskModifierKeyValue;
  }
}

inline void AddLtTaskModifier(double value, int* combined_key_value) {
  if (combined_key_value != nullptr && IsLtTaskModifierPressed(value)) {
    *combined_key_value |= kLtTaskModifierKeyValue;
  }
}

}  // namespace runner

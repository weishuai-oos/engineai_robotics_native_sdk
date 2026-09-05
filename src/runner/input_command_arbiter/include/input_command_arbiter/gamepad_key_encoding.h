#pragma once

#include <cmath>

namespace runner {

// The pre-built SDK task parser exposes only digital key names. The two
// horizontal d-pad directions cannot be pressed together by the hardware, so
// the input adapters use both bits as a private LT marker. task_motion maps
// [CROSS_Y_LEFT, CROSS_Y_RIGHT, key] to the physical LT + key combination.
// Keep these indices aligned with GamepadTool::kKeyString in the SDK core ABI.
inline constexpr int kLtTaskModifierLeftIndex = 10;   // CROSS_Y_LEFT
inline constexpr int kLtTaskModifierRightIndex = 11;  // CROSS_Y_RIGHT
inline constexpr int kLtTaskModifierKeyValue =
    (1 << kLtTaskModifierLeftIndex) | (1 << kLtTaskModifierRightIndex);
inline constexpr double kLtTaskModifierThreshold = 0.5;

inline bool IsLtTaskModifierPressed(double value) {
  return std::isfinite(value) && value >= kLtTaskModifierThreshold;
}

inline void AddLtTaskModifier(double value, int* combined_key_value) {
  if (combined_key_value != nullptr && IsLtTaskModifierPressed(value)) {
    *combined_key_value |= kLtTaskModifierKeyValue;
  }
}

}  // namespace runner

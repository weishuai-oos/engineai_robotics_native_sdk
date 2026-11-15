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
// Reserve a complete key combination outside the 1..50 dual-trigger action
// encodings for the otherwise indistinguishable physical RT + D-pad up.
// As with the existing trigger markers, this value is internal to the task
// parser; it does not alter the input message or the physical button fields.
// LT + RT + BACK + START shares this reserved value and must not be bound.
inline constexpr int kRtUpTaskKeyValue =
    kRtTaskModifierKeyValue | kLtTaskModifierKeyValue | (1 << 6) | (1 << 7);
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
    // The other adapters and the debouncer call AddLtTaskModifier first.
    // Only the unmodified RT + up combination gets the reserved value; all
    // existing combinations, including dual-trigger numbered tests, keep
    // their previous encoding. Reapplying the modifier is idempotent.
    if (*combined_key_value == (1 << kRtTaskModifierUpIndex)) {
      *combined_key_value = kRtUpTaskKeyValue;
    } else {
      *combined_key_value |= kRtTaskModifierKeyValue;
    }
  }
}

inline void AddLtTaskModifier(double value, int* combined_key_value) {
  if (combined_key_value != nullptr && IsLtTaskModifierPressed(value)) {
    *combined_key_value |= kLtTaskModifierKeyValue;
  }
}

}  // namespace runner

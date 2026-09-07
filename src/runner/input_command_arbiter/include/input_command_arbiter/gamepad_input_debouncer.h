#pragma once

#include <array>
#include <chrono>
#include <cstddef>

#include "gamepad_info/gamepad_info.h"

namespace runner {

// Debounces digital command inputs after hardware/virtual sources have been
// arbitrated. Presses require a short stable interval; releases are immediate
// so a stop or safety release is never delayed.
class GamepadInputDebouncer {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit GamepadInputDebouncer(
      std::chrono::milliseconds press_debounce = std::chrono::milliseconds(30));

  void Reset();
  data::GamepadInfo Update(const data::GamepadInfo& raw);

  // Exposed for deterministic unit tests; production code should use Update().
  data::GamepadInfo UpdateAt(const data::GamepadInfo& raw, TimePoint now);

 private:
  using ButtonMember = int data::GamepadInfo::*;

  int DebounceButton(int raw, int* stable, int* candidate, TimePoint* candidate_since, TimePoint now);
  int DebounceDirection(int raw, int* stable, int* candidate, TimePoint* candidate_since, TimePoint now);
  int BuildCombinedKeyValue(const data::GamepadInfo& input, bool lt_pressed, bool rt_pressed);

  static constexpr std::array<ButtonMember, 8> kButtonMembers = {
      &data::GamepadInfo::LB,   &data::GamepadInfo::RB,    &data::GamepadInfo::A,
      &data::GamepadInfo::B,    &data::GamepadInfo::X,     &data::GamepadInfo::Y,
      &data::GamepadInfo::BACK, &data::GamepadInfo::START,
  };

  std::chrono::milliseconds press_debounce_;
  std::array<int, kButtonMembers.size()> stable_buttons_{};
  std::array<int, kButtonMembers.size()> candidate_buttons_{};
  std::array<TimePoint, kButtonMembers.size()> candidate_button_since_{};
  int stable_cross_x_ = 0;
  int candidate_cross_x_ = 0;
  TimePoint candidate_cross_x_since_{};
  int stable_cross_y_ = 0;
  int candidate_cross_y_ = 0;
  TimePoint candidate_cross_y_since_{};
  int stable_lt_pressed_ = 0;
  int candidate_lt_pressed_ = 0;
  TimePoint candidate_lt_since_{};
  int stable_rt_pressed_ = 0;
  int candidate_rt_pressed_ = 0;
  TimePoint candidate_rt_since_{};
  data::GamepadTool gamepad_tool_;
};

}  // namespace runner

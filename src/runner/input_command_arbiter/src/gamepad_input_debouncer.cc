#include "input_command_arbiter/gamepad_input_debouncer.h"

#include "input_command_arbiter/gamepad_key_encoding.h"

namespace runner {

GamepadInputDebouncer::GamepadInputDebouncer(std::chrono::milliseconds press_debounce)
    : press_debounce_(press_debounce) {
  Reset();
}

void GamepadInputDebouncer::Reset() {
  stable_buttons_.fill(0);
  candidate_buttons_.fill(0);
  candidate_button_since_.fill(TimePoint{});
  stable_cross_x_ = 0;
  candidate_cross_x_ = 0;
  candidate_cross_x_since_ = TimePoint{};
  stable_cross_y_ = 0;
  candidate_cross_y_ = 0;
  candidate_cross_y_since_ = TimePoint{};
  stable_lt_pressed_ = 0;
  candidate_lt_pressed_ = 0;
  candidate_lt_since_ = TimePoint{};
  stable_rt_pressed_ = 0;
  candidate_rt_pressed_ = 0;
  candidate_rt_since_ = TimePoint{};
}

data::GamepadInfo GamepadInputDebouncer::Update(const data::GamepadInfo& raw) {
  return UpdateAt(raw, Clock::now());
}

data::GamepadInfo GamepadInputDebouncer::UpdateAt(const data::GamepadInfo& raw, TimePoint now) {
  data::GamepadInfo filtered = raw;
  for (std::size_t i = 0; i < kButtonMembers.size(); ++i) {
    filtered.*kButtonMembers[i] =
        DebounceButton(raw.*kButtonMembers[i], &stable_buttons_[i], &candidate_buttons_[i],
                       &candidate_button_since_[i], now);
  }
  filtered.CROSS_X =
      DebounceDirection(raw.CROSS_X, &stable_cross_x_, &candidate_cross_x_, &candidate_cross_x_since_, now);
  filtered.CROSS_Y =
      DebounceDirection(raw.CROSS_Y, &stable_cross_y_, &candidate_cross_y_, &candidate_cross_y_since_, now);

  const int raw_lt_pressed = IsLtTaskModifierPressed(raw.LT) ? 1 : 0;
  const int raw_rt_pressed = IsRtTaskModifierPressed(raw.RT) ? 1 : 0;
  const bool lt_pressed = DebounceButton(raw_lt_pressed, &stable_lt_pressed_, &candidate_lt_pressed_,
                                         &candidate_lt_since_, now) != 0;
  const bool rt_pressed = DebounceButton(raw_rt_pressed, &stable_rt_pressed_, &candidate_rt_pressed_,
                                         &candidate_rt_since_, now) != 0;
  filtered.combined_key_value = BuildCombinedKeyValue(filtered, lt_pressed, rt_pressed);
  return filtered;
}

int GamepadInputDebouncer::DebounceButton(int raw, int* stable, int* candidate, TimePoint* candidate_since,
                                          TimePoint now) {
  const int normalized_raw = raw != 0 ? 1 : 0;
  if (normalized_raw == *stable) {
    *candidate = normalized_raw;
    *candidate_since = now;
    return *stable;
  }
  if (normalized_raw == 0) {
    *stable = 0;
    *candidate = 0;
    *candidate_since = now;
    return 0;
  }
  if (*candidate != normalized_raw) {
    *candidate = normalized_raw;
    *candidate_since = now;
    return *stable;
  }
  if (now - *candidate_since >= press_debounce_) {
    *stable = normalized_raw;
  }
  return *stable;
}

int GamepadInputDebouncer::DebounceDirection(int raw, int* stable, int* candidate, TimePoint* candidate_since,
                                             TimePoint now) {
  if (raw == *stable) {
    *candidate = raw;
    *candidate_since = now;
    return *stable;
  }
  if (raw == 0) {
    *stable = 0;
    *candidate = 0;
    *candidate_since = now;
    return 0;
  }
  // When changing direction, release the old direction immediately and wait
  // for the new one. This prevents a d-pad bounce from keeping an old action.
  if (*stable != 0 && raw != *stable) {
    *stable = 0;
    *candidate = raw;
    *candidate_since = now;
    return 0;
  }
  if (*candidate != raw) {
    *candidate = raw;
    *candidate_since = now;
    return *stable;
  }
  if (now - *candidate_since >= press_debounce_) {
    *stable = raw;
  }
  return *stable;
}

int GamepadInputDebouncer::BuildCombinedKeyValue(const data::GamepadInfo& input, bool lt_pressed,
                                                 bool rt_pressed) {
  int value = 0;
  if (input.LB) value |= gamepad_tool_.KeyStringToValue("LB");
  if (input.RB) value |= gamepad_tool_.KeyStringToValue("RB");
  if (input.A) value |= gamepad_tool_.KeyStringToValue("A");
  if (input.B) value |= gamepad_tool_.KeyStringToValue("B");
  if (input.X) value |= gamepad_tool_.KeyStringToValue("X");
  if (input.Y) value |= gamepad_tool_.KeyStringToValue("Y");
  if (input.BACK) value |= gamepad_tool_.KeyStringToValue("BACK");
  if (input.START) value |= gamepad_tool_.KeyStringToValue("START");
  if (input.CROSS_X > 0) value |= gamepad_tool_.KeyStringToValue("CROSS_X_UP");
  if (input.CROSS_X < 0) value |= gamepad_tool_.KeyStringToValue("CROSS_X_DOWN");
  if (input.CROSS_Y > 0) value |= gamepad_tool_.KeyStringToValue("CROSS_Y_LEFT");
  if (input.CROSS_Y < 0) value |= gamepad_tool_.KeyStringToValue("CROSS_Y_RIGHT");
  AddLtTaskModifier(lt_pressed ? 1.0 : 0.0, &value);
  AddRtTaskModifier(rt_pressed ? 1.0 : 0.0, &value);
  return value;
}

}  // namespace runner

#include "input_command_arbiter/gamepad_input_debouncer.h"

#include <gtest/gtest.h>

#include "input_command_arbiter/gamepad_key_encoding.h"

namespace runner {
namespace {

TEST(GamepadInputDebouncerTest, RequiresStablePressAndReleasesImmediately) {
  GamepadInputDebouncer debouncer(std::chrono::milliseconds(30));
  data::GamepadInfo input;
  input.LB = 1;
  const auto t0 = GamepadInputDebouncer::Clock::time_point{};

  EXPECT_EQ(debouncer.UpdateAt(input, t0).LB, 0);
  EXPECT_EQ(debouncer.UpdateAt(input, t0 + std::chrono::milliseconds(29)).LB, 0);
  EXPECT_EQ(debouncer.UpdateAt(input, t0 + std::chrono::milliseconds(30)).LB, 1);

  input.LB = 0;
  EXPECT_EQ(debouncer.UpdateAt(input, t0 + std::chrono::milliseconds(31)).LB, 0);
}

TEST(GamepadInputDebouncerTest, DebouncesLtActionModifierWithoutChangingAnalogValue) {
  GamepadInputDebouncer debouncer(std::chrono::milliseconds(30));
  data::GamepadInfo input;
  input.LT = 1.0;
  input.A = 1;
  const auto t0 = GamepadInputDebouncer::Clock::time_point{};

  const auto before_stable = debouncer.UpdateAt(input, t0);
  EXPECT_DOUBLE_EQ(before_stable.LT, 1.0);
  EXPECT_EQ(before_stable.combined_key_value, 0);

  const auto stable = debouncer.UpdateAt(input, t0 + std::chrono::milliseconds(30));
  EXPECT_EQ(stable.A, 1);
  data::GamepadTool gamepad_tool;
  const int expected = gamepad_tool.KeyStringToValue("A") | kLtTaskModifierKeyValue;
  EXPECT_EQ(stable.combined_key_value, expected);
}

}  // namespace
}  // namespace runner

#include "gamepad/gamepad_driver.h"
#include "input_command_arbiter/input_source_selection.h"

#include <sys/stat.h>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace {

void CheckMapping() {
  hardware::GamepadEventMapping mapping;
  js_event event{};
  event.type = JS_EVENT_AXIS | JS_EVENT_INIT;
  event.number = 2;
  assert(mapping.LogicalKey(event) == JSKEY_LT);

  mapping.axes_available = true;
  mapping.axes[2] = ABS_RX;
  assert(mapping.LogicalKey(event) == JSKEY_RIGHTSTICK_X);
  mapping.axes[4] = ABS_HAT0Y;
  event.number = 4;
  assert(mapping.LogicalKey(event) == JSKEY_CROSS_Y);
  mapping.axes[5] = ABS_BRAKE;
  mapping.axes[6] = ABS_GAS;
  mapping.axes[2] = ABS_Z;
  event.number = 2;
  assert(mapping.LogicalKey(event) == JSKEY_RIGHTSTICK_X);
  event.number = 5;
  assert(mapping.LogicalKey(event) == JSKEY_LT);

  mapping.buttons_available = true;
  mapping.buttons[0] = BTN_START;
  event = {};
  event.type = JS_EVENT_BUTTON | JS_EVENT_INIT;
  assert(mapping.LogicalKey(event) == JSKEY_START);
  mapping.buttons[1] = BTN_SOUTH;
  event.number = 1;
  assert(mapping.LogicalKey(event) == JSKEY_A);
  mapping.buttons[2] = BTN_DPAD_UP;
  event.number = 2;
  assert(mapping.LogicalKey(event) == hardware::kGamepadDpadUp);
  mapping.buttons[3] = BTN_TOUCH;
  event.number = 3;
  assert(mapping.LogicalKey(event) == -1);
  mapping.buttons[4] = BTN_X;
  event.number = 4;
  assert(mapping.LogicalKey(event) == JSKEY_X);
  mapping.buttons[5] = BTN_Y;
  event.number = 5;
  assert(mapping.LogicalKey(event) == JSKEY_Y);
  mapping.buttons[6] = BTN_TL2;
  event.number = 6;
  assert(mapping.LogicalKey(event) == hardware::kGamepadTriggerLeftButton);
}

void CheckSourceSelection() {
  using runner::InputSourceSelection;
  using runner::ParseInputSourceSelection;
  assert(ParseInputSourceSelection(nullptr) == InputSourceSelection::kAutomatic);
  assert(ParseInputSourceSelection("") == InputSourceSelection::kAutomatic);
  assert(ParseInputSourceSelection("auto") == InputSourceSelection::kAutomatic);
  assert(ParseInputSourceSelection("gamepad") == InputSourceSelection::kGamepadOnly);
  bool rejected = false;
  try {
    ParseInputSourceSelection("typo");
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  assert(rejected);
}

void WriteEvent(int fd, unsigned char type, unsigned char number, short value) {
  js_event event{};
  event.type = type;
  event.number = number;
  event.value = value;
  assert(write(fd, &event, sizeof(event)) == sizeof(event));
}

std::size_t OpenFileCount() {
  std::size_t count = 0;
  for (const auto& entry : std::filesystem::directory_iterator("/proc/self/fd")) {
    static_cast<void>(entry);
    ++count;
  }
  return count;
}

void CheckIndependentDevices() {
  char directory_template[] = "/tmp/engineai-gamepad-test-XXXXXX";
  char* directory = mkdtemp(directory_template);
  assert(directory);
  const std::string first = std::string(directory) + "/first";
  const std::string second = std::string(directory) + "/second";
  assert(mkfifo(first.c_str(), 0600) == 0);
  assert(mkfifo(second.c_str(), 0600) == 0);
  const auto files_before = OpenFileCount();
  {
    hardware::LogitechGamepadDriver player_one(first);
    hardware::LogitechGamepadDriver player_two(second);
    assert(player_one.Init() == EXIT_SUCCESS);
    assert(player_two.Init() == EXIT_SUCCESS);
    const int writer_one = open(first.c_str(), O_WRONLY | O_NONBLOCK);
    const int writer_two = open(second.c_str(), O_WRONLY | O_NONBLOCK);
    assert(writer_one >= 0 && writer_two >= 0);

    WriteEvent(writer_one, JS_EVENT_BUTTON | JS_EVENT_INIT, 0, 1);
    WriteEvent(writer_one, JS_EVENT_AXIS | JS_EVENT_INIT, 1, -32768);
    WriteEvent(writer_one, JS_EVENT_AXIS | JS_EVENT_INIT, 2, -32768);
    WriteEvent(writer_two, JS_EVENT_BUTTON, 1, 1);
    WriteEvent(writer_two, JS_EVENT_AXIS, 1, 32767);
    WriteEvent(writer_two, JS_EVENT_AXIS, 5, 32767);
    assert(player_one.ListenInput() == EXIT_SUCCESS);
    assert(player_two.ListenInput() == EXIT_SUCCESS);
    assert(player_one.logitech_data_res.A == 1 && player_one.logitech_data_res.B == 0);
    assert(player_two.logitech_data_res.B == 1 && player_two.logitech_data_res.A == 0);
    assert(player_one.logitech_data_res.leftStickXAnalog == 1.0f);
    assert(player_two.logitech_data_res.leftStickXAnalog == -1.0f);
    assert(player_one.logitech_data_res.LT == 0.0f);
    assert(player_two.logitech_data_res.RT == 1.0f);
    // An empty nonblocking device preserves the held state.
    assert(player_one.ListenInput() == EXIT_SUCCESS && player_one.logitech_data_res.A == 1);

    close(writer_one);
    assert(player_one.ListenInput() == EXIT_FAILURE);
    assert(player_one.logitech_data_res.A == 0 && player_one.logitech_data_res.leftStickXAnalog == 0.0f);
    assert(player_two.ListenInput() == EXIT_SUCCESS && player_two.logitech_data_res.B == 1);
    assert(player_one.Init() == EXIT_SUCCESS);
    const int reconnected_writer = open(first.c_str(), O_WRONLY | O_NONBLOCK);
    assert(reconnected_writer >= 0);
    WriteEvent(reconnected_writer, JS_EVENT_BUTTON, 2, 1);
    assert(player_one.ListenInput() == EXIT_SUCCESS && player_one.logitech_data_res.X == 1);
    assert(player_two.logitech_data_res.X == 0);
    close(reconnected_writer);
    close(writer_two);
    // Reinitialization replaces, rather than leaks, the open descriptor.
    for (int i = 0; i < 10; ++i) assert(player_two.Init() == EXIT_SUCCESS);
  }
  assert(OpenFileCount() == files_before);
  unlink(first.c_str());
  unlink(second.c_str());
  rmdir(directory);
}

void CheckDeviceEnvironment() {
  const char* original = std::getenv("ENGINEAI_GAMEPAD_DEVICE");
  const bool had_original = original != nullptr;
  const std::string original_value = original ? original : "";
  unsetenv("ENGINEAI_GAMEPAD_DEVICE");
  assert(hardware::LogitechGamepadDriver().DevicePath() == "/dev/input/js0");
  setenv("ENGINEAI_GAMEPAD_DEVICE", "/dev/input/by-id/player-two-joystick", 1);
  assert(hardware::LogitechGamepadDriver().DevicePath() == "/dev/input/by-id/player-two-joystick");
  assert(hardware::LogitechGamepadDriver("/explicit/device").DevicePath() == "/explicit/device");
  if (had_original) setenv("ENGINEAI_GAMEPAD_DEVICE", original_value.c_str(), 1);
  else unsetenv("ENGINEAI_GAMEPAD_DEVICE");
}

}  // namespace

int main() {
  static_assert(!std::is_copy_constructible_v<hardware::LogitechGamepadDriver>);
  static_assert(!std::is_move_constructible_v<hardware::LogitechGamepadDriver>);
  CheckMapping();
  CheckSourceSelection();
  CheckDeviceEnvironment();
  CheckIndependentDevices();
  std::cout << "Gamepad binding, mapping, input source and reconnect checks passed\n";
}

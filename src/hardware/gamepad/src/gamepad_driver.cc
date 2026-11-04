/* Copyright (C) 2021 - ~, SCUT-RobotLab Development Team
 * Linux joystick input with the original F710/XInput control semantics.
 * ENGINEAI_GAMEPAD_DEVICE can bind an instance to a stable by-id device path.
 */

#include "gamepad/gamepad_driver.h"

#include <sys/ioctl.h>

#include <algorithm>
#include <utility>

namespace hardware {
namespace {

std::string GamepadDeviceFromEnvironment() {
  const char* device = std::getenv("ENGINEAI_GAMEPAD_DEVICE");
  return device && *device ? device : "/dev/input/js0";
}

}  // namespace

LogitechGamepadDriver::LogitechGamepadDriver()
    : LogitechGamepadDriver(GamepadDeviceFromEnvironment()) {}

LogitechGamepadDriver::LogitechGamepadDriver(std::string device_path)
    : dev(std::move(device_path)) {
  Zeros();
}

LogitechGamepadDriver::~LogitechGamepadDriver() { Close(); }

void LogitechGamepadDriver::Close() {
  if (fd >= 0) {
    close(fd);
    fd = -1;
  }
  Zeros();
}

int LogitechGamepadDriver::Init() {
  Close();
  fd = open(dev.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) return EXIT_FAILURE;

  event_mapping_ = {};
  event_mapping_.axes_available = ioctl(fd, JSIOCGAXMAP, event_mapping_.axes.data()) >= 0;
  event_mapping_.buttons_available = ioctl(fd, JSIOCGBTNMAP, event_mapping_.buttons.data()) >= 0;
  return EXIT_SUCCESS;
}

void LogitechGamepadDriver::Zeros() {
  Keystate_map.clear();
  for (const int key : {JSKEY_A, JSKEY_B, JSKEY_X, JSKEY_Y, JSKEY_LB, JSKEY_RB,
                        JSKEY_BACK, JSKEY_START, JSKEY_HOME, JSKEY_CROSS_X, JSKEY_CROSS_Y,
                        JSKEY_LT, JSKEY_RT, JSKEY_LEFTSTICK_X, JSKEY_LEFTSTICK_Y,
                        JSKEY_RIGHTSTICK_X, JSKEY_RIGHTSTICK_Y}) {
    Keystate_map[key] = 0;
  }
  logitech_data_res = {};
}

void LogitechGamepadDriver::ProcessEvent(const js_event& event) {
  const int key = event_mapping_.LogicalKey(event);
  if (key < 0) return;

  const int value = event.value;
  if (key == kGamepadTriggerLeftButton || key == kGamepadTriggerRightButton) {
    const float trigger = value != 0 ? 1.0f : 0.0f;
    if (key == kGamepadTriggerLeftButton) {
      logitech_data_res.LT = trigger;
      Keystate_map[JSKEY_LT] = static_cast<int>(1000.0f * trigger);
    } else {
      logitech_data_res.RT = trigger;
      Keystate_map[JSKEY_RT] = static_cast<int>(1000.0f * trigger);
    }
    return;
  }
  if (key == JSKEY_LT || key == JSKEY_RT) {
    const float trigger = static_cast<float>(value + 32768) / 65535.0f;
    Keystate_map[key] = static_cast<int>(1000.0f * trigger);
    if (key == JSKEY_LT) logitech_data_res.LT = trigger;
    else logitech_data_res.RT = trigger;
    return;
  }

  if (key == JSKEY_LEFTSTICK_X || key == JSKEY_LEFTSTICK_Y ||
      key == JSKEY_RIGHTSTICK_X || key == JSKEY_RIGHTSTICK_Y) {
    const float stick = std::clamp(-static_cast<float>(value) / 32767.0f, -1.0f, 1.0f);
    Keystate_map[key] = static_cast<int>(1000.0f * stick);
    // The SDK names X as forward/backward and Y as left/right.
    if (key == JSKEY_LEFTSTICK_X) logitech_data_res.leftStickYAnalog = stick;
    else if (key == JSKEY_LEFTSTICK_Y) logitech_data_res.leftStickXAnalog = stick;
    else if (key == JSKEY_RIGHTSTICK_X) logitech_data_res.rightStickYAnalog = stick;
    else logitech_data_res.rightStickXAnalog = stick;
    return;
  }

  if (key == JSKEY_CROSS_X || key == JSKEY_CROSS_Y) {
    const int direction = value < 0 ? 1 : (value > 0 ? -1 : 0);
    Keystate_map[key] = direction;
    if (key == JSKEY_CROSS_X) logitech_data_res.CROSS_Y = direction;
    else logitech_data_res.CROSS_X = direction;
    return;
  }

  const int pressed = value != 0;
  Keystate_map[key] = pressed;
  switch (key) {
    case JSKEY_A: logitech_data_res.A = pressed; break;
    case JSKEY_B: logitech_data_res.B = pressed; break;
    case JSKEY_X: logitech_data_res.X = pressed; break;
    case JSKEY_Y: logitech_data_res.Y = pressed; break;
    case JSKEY_LB: logitech_data_res.LB = pressed; break;
    case JSKEY_RB: logitech_data_res.RB = pressed; break;
    case JSKEY_BACK: logitech_data_res.BACK = pressed; break;
    case JSKEY_START: logitech_data_res.START = pressed; break;
    case JSKEY_HOME: logitech_data_res.HOME = pressed; break;
    case 0x901: logitech_data_res.leftStickButton = pressed; break;
    case 0xa01: logitech_data_res.rightStickButton = pressed; break;
    case kGamepadDpadUp:
    case kGamepadDpadDown:
      logitech_data_res.CROSS_X = Keystate_map[kGamepadDpadUp] - Keystate_map[kGamepadDpadDown];
      break;
    case kGamepadDpadLeft:
    case kGamepadDpadRight:
      logitech_data_res.CROSS_Y = Keystate_map[kGamepadDpadLeft] - Keystate_map[kGamepadDpadRight];
      break;
    default: break;
  }
}

int LogitechGamepadDriver::ListenInput() {
  if (fd < 0) {
    Zeros();
    return EXIT_FAILURE;
  }
  // Bound work per control tick while draining initialization and input bursts.
  for (int count = 0; count < 64; ++count) {
    js_event event{};
    const ssize_t bytes = read(fd, &event, sizeof(event));
    if (bytes == sizeof(event)) {
      ProcessEvent(event);
      continue;
    }
    if (bytes < 0 && errno == EINTR) continue;
    if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return EXIT_SUCCESS;
    // EOF, a truncated event or a device error must release all controls.
    // The adapter retries this same device path, preserving player ownership.
    Close();
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

}  // namespace hardware

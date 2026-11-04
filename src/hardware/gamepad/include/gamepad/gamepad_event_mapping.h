#pragma once

#include <linux/input.h>
#include <linux/joystick.h>

#include <algorithm>
#include <array>

namespace hardware {

// Logical event identifiers retain the original F710/XInput layout.
constexpr int kGamepadDpadUp = 0x10001;
constexpr int kGamepadDpadDown = 0x10002;
constexpr int kGamepadDpadLeft = 0x10003;
constexpr int kGamepadDpadRight = 0x10004;
constexpr int kGamepadTriggerLeftButton = 0x10005;
constexpr int kGamepadTriggerRightButton = 0x10006;

struct GamepadEventMapping {
  std::array<unsigned char, ABS_CNT> axes{};
  std::array<unsigned short, KEY_MAX - BTN_MISC + 1> buttons{};
  bool axes_available = false;
  bool buttons_available = false;

  int LogicalKey(const js_event& event) const {
    const int type = event.type & ~JS_EVENT_INIT;
    if (type == JS_EVENT_AXIS) {
      if (!axes_available) return (event.number << 8) | JS_EVENT_AXIS;
      if (event.number >= axes.size()) return -1;
      // Some Bluetooth Xbox drivers expose the right stick as Z/RZ and
      // triggers as BRAKE/GAS; xpad exposes RX/RY and Z/RZ respectively.
      const bool brake_gas_triggers =
          std::find(axes.begin(), axes.end(), ABS_BRAKE) != axes.end() &&
          std::find(axes.begin(), axes.end(), ABS_GAS) != axes.end();
      switch (axes[event.number]) {
        case ABS_X: return 0x002;
        case ABS_Y: return 0x102;
        case ABS_RX: return 0x302;
        case ABS_RY: return 0x402;
        case ABS_Z: return brake_gas_triggers ? 0x302 : 0x202;
        case ABS_RZ: return brake_gas_triggers ? 0x402 : 0x502;
        case ABS_BRAKE: return 0x202;
        case ABS_GAS: return 0x502;
        case ABS_HAT0X: return 0x602;
        case ABS_HAT0Y: return 0x702;
        default: return -1;
      }
    }
    if (type != JS_EVENT_BUTTON) return -1;
    if (!buttons_available) return (event.number << 8) | JS_EVENT_BUTTON;
    if (event.number >= buttons.size()) return -1;
    switch (buttons[event.number]) {
      case BTN_SOUTH: return 0x001;
      case BTN_EAST: return 0x101;
      // xpad and F710 use the historical BTN_X/BTN_Y codes (whose
      // BTN_NORTH/BTN_WEST aliases do not match the Xbox printed positions).
      case BTN_X: return 0x201;
      case BTN_Y: return 0x301;
      case BTN_TL: return 0x401;
      case BTN_TR: return 0x501;
      case BTN_TL2: return kGamepadTriggerLeftButton;
      case BTN_TR2: return kGamepadTriggerRightButton;
      case BTN_SELECT: return 0x601;
      case BTN_START: return 0x701;
      case BTN_MODE: return 0x801;
      case BTN_THUMBL: return 0x901;
      case BTN_THUMBR: return 0xa01;
      case BTN_DPAD_UP: return kGamepadDpadUp;
      case BTN_DPAD_DOWN: return kGamepadDpadDown;
      case BTN_DPAD_LEFT: return kGamepadDpadLeft;
      case BTN_DPAD_RIGHT: return kGamepadDpadRight;
      default: return -1;
    }
  }
};

}  // namespace hardware

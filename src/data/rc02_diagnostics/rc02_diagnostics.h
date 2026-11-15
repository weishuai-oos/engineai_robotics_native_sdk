#pragma once

#include <array>
#include <cstdint>

namespace data {

// Raw RC02 receiver frame plus normalized diagnostic values. The source
// timestamp uses the host steady clock and is never compared directly with
// another machine's steady clock.
struct Rc02Diagnostics {
  // Exact validated RC02 wire frame (header, length, control payload, CRC).
  std::array<uint8_t, 25> raw_frame{};
  uint8_t raw_frame_size = 0;
  int64_t source_monotonic_ns = 0;
  bool connected = false;
  bool frame_valid = false;
  uint8_t control_id = 0;
  uint8_t heartbeat = 0;
  uint8_t l2_raw = 0;
  uint8_t r2_raw = 0;
  int8_t left_stick_ud_raw = 0;
  int8_t left_stick_lr_raw = 0;
  int8_t right_stick_ud_raw = 0;
  int8_t right_stick_lr_raw = 0;
  uint8_t buttons_1 = 0;
  uint8_t buttons_2 = 0;
  uint8_t buttons_3 = 0;
  uint8_t motion_command = 0;
  uint8_t skill_command = 0;
  std::array<uint8_t, 8> reserved{};
  std::array<double, 6> normalized_analog{};
};

}  // namespace data

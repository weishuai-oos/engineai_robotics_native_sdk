#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace data {

// One runner-step snapshot of the Leo command speed chain. The ROS publisher
// can measure its age with the same host's steady_clock; a recorder on another
// machine must not subtract this from its own monotonic clock.
struct LeoCommandDiagnostics {
  int64_t source_monotonic_ns = 0;
  std::string source_param_tag;
  bool active = false;
  std::array<double, 3> raw_stick{};
  std::array<double, 3> target_tactical{};
  std::array<double, 3> shaped_tactical{};
  std::array<double, 3> policy_command{};
};

}  // namespace data

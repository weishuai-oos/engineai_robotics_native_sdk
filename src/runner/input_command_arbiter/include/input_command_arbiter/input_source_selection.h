#pragma once

#include <stdexcept>
#include <string_view>

namespace runner {

enum class InputSourceSelection { kAutomatic, kGamepadOnly, kVirtualOnly };

inline InputSourceSelection ParseInputSourceSelection(const char* source) {
  if (!source || !*source || std::string_view(source) == "auto") {
    return InputSourceSelection::kAutomatic;
  }
  if (std::string_view(source) == "gamepad") return InputSourceSelection::kGamepadOnly;
  if (std::string_view(source) == "virtual") return InputSourceSelection::kVirtualOnly;
  throw std::invalid_argument("ENGINEAI_INPUT_SOURCE must be 'auto', 'gamepad' or 'virtual'");
}

}  // namespace runner

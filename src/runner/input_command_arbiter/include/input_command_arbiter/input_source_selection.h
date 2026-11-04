#pragma once

#include <stdexcept>
#include <string_view>

namespace runner {

enum class InputSourceSelection { kAutomatic, kGamepadOnly };

inline InputSourceSelection ParseInputSourceSelection(const char* source) {
  if (!source || !*source || std::string_view(source) == "auto") {
    return InputSourceSelection::kAutomatic;
  }
  if (std::string_view(source) == "gamepad") return InputSourceSelection::kGamepadOnly;
  throw std::invalid_argument("ENGINEAI_INPUT_SOURCE must be 'auto' or 'gamepad'");
}

}  // namespace runner

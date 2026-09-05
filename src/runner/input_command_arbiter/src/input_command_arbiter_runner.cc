#include "input_command_arbiter/input_command_arbiter_runner.h"
#include "input_command_arbiter/gamepad_input_adapter.h"
#include "input_command_arbiter/rc02_input_adapter.h"
#include "input_command_arbiter/virtual_gamepad_input_adapter.h"

#include <glog/logging.h>

namespace runner {

InputCommandArbiterRunner::InputCommandArbiterRunner(std::string_view name,
                                                     const std::shared_ptr<data::DataStore>& data_store)
    : BasicRunner(name, data_store) {
  RegisterHardwareSource("rc02", std::make_shared<Rc02InputAdapter>("rc02", data_store));
  RegisterHardwareSource("gamepad", std::make_shared<GamepadInputAdapter>("gamepad", data_store));
  RegisterOverrideSource("virtual_gamepad",
                         std::make_shared<VirtualGamepadInputAdapter>("virtual_gamepad", data_store));

  selected_hardware_idx_ = -1;
  hardware_input_publisher_ =
      data::VariantStore::GetInstance().CreatePublisher<data::GamepadInfo>("hardware/gamepad_info");
  for (int i = 0; i < static_cast<int>(hardware_sources_.size()); ++i) {
    if (!hardware_sources_[i]->Init()) {
      LOG(WARNING) << "InputCommandArbiterRunner: Init failed for hardware source '"
                   << hardware_sources_[i]->GetName() << "'.";
      continue;
    }
    LOG(INFO) << "InputCommandArbiterRunner: hardware source '" << hardware_sources_[i]->GetName()
              << "' Init succeeded at index " << i << ".";
  }

  for (auto& source : override_sources_) {
    if (!source->Init()) {
      LOG(WARNING) << "InputCommandArbiterRunner: Init failed for override source '" << source->GetName() << "'.";
    }
  }
}

void InputCommandArbiterRunner::Run() {
  data::GamepadInfo result;
  result.Reset();

  // Service every adapter so a failed startup ACK cannot permanently suppress
  // RC02 retries after F710 was selected. RC02 has priority once fresh input
  // arrives; a merely open port or pending handshake never claims control.
  int active_hardware_idx = -1;
  for (int i = 0; i < static_cast<int>(hardware_sources_.size()); ++i) {
    static_cast<void>(hardware_sources_[i]->Run());
    if (active_hardware_idx < 0 && hardware_sources_[i]->IsActive()) {
      active_hardware_idx = i;
      hardware_sources_[i]->Process(result);
    }
  }
  if (active_hardware_idx != selected_hardware_idx_) {
    selected_hardware_idx_ = active_hardware_idx;
    if (selected_hardware_idx_ >= 0) {
      LOG(INFO) << "Hardware source locked: " << hardware_sources_[selected_hardware_idx_]->GetName();
    }
  }
  // Publish only the selected hardware snapshot; disconnected fallback
  // adapters must not overwrite it for ROS2 and other hardware consumers.
  hardware_input_publisher_.Publish(result);

  for (auto& source : override_sources_) {
    static_cast<void>(source->Run());
    if (source->IsActive()) {
      source->Process(result);
    }
  }

  data_store_->gamepad_info.Set(result);
}

void InputCommandArbiterRunner::RegisterHardwareSource(const std::string& name,
                                                     std::shared_ptr<BaseInputAdapter> adapter) {
  hardware_sources_.emplace_back(std::move(adapter));
  LOG(INFO) << "Registered hardware source: " << name;
}

void InputCommandArbiterRunner::RegisterOverrideSource(const std::string& name,
                                                       std::shared_ptr<BaseInputAdapter> adapter) {
  override_sources_.emplace_back(std::move(adapter));
  LOG(INFO) << "Registered override source: " << name;
}

}  // namespace runner

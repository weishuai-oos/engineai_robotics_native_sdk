#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "basic/basic_runner.h"
#include "basic/runner_registry.h"
#include "data/variant_store/variant_store.h"
#include "motor_debug/motor_debug.h"
#include "t800_safety/t800_safety.h"

namespace runner {

class T800SafetyRunner final : public BasicRunner {
 public:
  T800SafetyRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store)
      : BasicRunner(name, data_store) {}

  bool Initialize() override;
  bool Enter() override;
  void Run() override;
  TransitionState TryExit() override { return TransitionState::kCompleted; }
  bool Exit() override { return true; }
  void End() override {}

 private:
  bool initialized_ = false;
  t800_safety::T800JointEnvelope envelope_;
  std::unique_ptr<t800_safety::HeadJointHealthMonitor> health_;
  std::unique_ptr<t800_safety::JointCommandSafetyGate> gate_;
  t800_safety::MotorEnableGuard motor_enable_guard_;
  data::Subscriber<data::MotorDebug> motor_debug_subscriber_;
  std::shared_ptr<const data::MotorDebug> last_motor_debug_sample_;
  std::uint64_t last_motor_debug_update_ns_ = 0;
  bool have_reported_status_ = false;
  t800_safety::SafetyStatus last_reported_status_ =
      t800_safety::SafetyStatus::kNormal;
  t800_safety::SafetyProfile last_reported_profile_ =
      t800_safety::SafetyProfile::kPolicy;
  t800_safety::MotorEnablePhase last_reported_motor_enable_phase_ =
      t800_safety::MotorEnablePhase::kReady;
  std::uint32_t last_reported_reason_mask_ = t800_safety::kReasonNone;
  std::string last_reported_motion_;
  std::uint64_t status_frames_since_report_ = 0;
};

class T800MotorPreDriverSafetyRunner final : public BasicRunner {
 public:
  T800MotorPreDriverSafetyRunner(std::string_view name,
                                const std::shared_ptr<data::DataStore>& data_store)
      : BasicRunner(name, data_store) {}

  bool Initialize() override;
  bool Enter() override;
  void Run() override;
  TransitionState TryExit() override { return TransitionState::kCompleted; }
  bool Exit() override { return true; }
  void End() override {}

 private:
  bool initialized_ = false;
  t800_safety::T800JointEnvelope envelope_;
  std::unique_ptr<t800_safety::MotorCommandSafetyGate> gate_;
};

}  // namespace runner

REGISTER_RUNNER(T800SafetyRunner, "t800_safety_runner", kResident)
REGISTER_RUNNER(T800MotorPreDriverSafetyRunner,
                "t800_motor_pre_driver_safety_runner", kResident)

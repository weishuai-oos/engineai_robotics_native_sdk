#pragma once

#include <string>

#include "basic/motion_runner.h"
#include "basic/runner_registry.h"
#include "idle_param/idle_param.h"
#include "motor_debug/motor_debug.h"
#include "variant_store/variant_store.h"

namespace runner {

class IdleRunner : public MotionRunner {
 public:
  IdleRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store)
      : MotionRunner(name, data_store) {}
  ~IdleRunner() = default;

  bool Enter() override;
  void Run() override;
  TransitionState TryExit() override;
  bool Exit() override;
  void End() override;
  void SetupContext() override;
  void TeardownContext() override;

  void Log();

 private:
  enum class MotorEnablePhase {
    kSynchronizingDisabled,
    kWaitingForEnabled,
  };

  enum class MotorFeedbackState {
    kUnavailable,
    kDisabled,
    kEnabled,
    kFault,
  };

  // Idle runs at 500 Hz in the T800 task graph, so this is a two-second
  // feedback timeout before the task enters the fault path.
  static constexpr int kNumTryExit = 1000;

  MotorFeedbackState ReadMotorFeedback(std::string* reason) const;
  bool RetryOrFault(const char* phase, const std::string& reason);

  int iter_ = 0;
  int num_try_exit_ = 0;
  MotorEnablePhase motor_enable_phase_ = MotorEnablePhase::kSynchronizingDisabled;
  data::Subscriber<data::MotorDebug> motor_debug_subscriber_;

  std::shared_ptr<data::IdleParam> param_;
};

}  // namespace runner

REGISTER_RUNNER(IdleRunner, "idle_runner", kMotion)

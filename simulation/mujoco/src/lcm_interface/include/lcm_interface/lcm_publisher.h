#ifndef LCM_INTERFACE_INCLUDE_LCM_INTERFACE_LCM_PUBLISHER_H_
#define LCM_INTERFACE_INCLUDE_LCM_INTERFACE_LCM_PUBLISHER_H_

#include <chrono>
#include <cstdint>
#include <memory>

#include <glog/logging.h>
#include <lcm/lcm-cpp.hpp>

#include "lcm_data/SimState.hpp"
#include "lcm_data/DynamicVectorDouble.hpp"
#include "lcm_interface/lcm_data_store.h"
#include "lcm_interface/periodic_task.h"
#include "lcm_param/lcm_param.h"

class LcmPublisher : public lcm_interface::PeriodicTask {
 public:
  LcmPublisher(const data::LcmParam& param, const std::shared_ptr<LcmDataStore>& lcm_data_store);
  ~LcmPublisher() override { TaskStop(); }

  void TaskInit() override;
  void TaskRun() override;

 private:
  // This is the generated LCM data::DynamicVectorDouble wire type. It is
  // intentionally distinct from ROS interface_protocol/msg/DynamicVectorDouble,
  // which has Header + float64[] and is never used on the LCM channel.
  using LcmControlDiagnostics = data::DynamicVectorDouble;

  std::shared_ptr<LcmDataStore> lcm_data_store_;
  SimState sim_state_;
  LcmControlDiagnostics lcm_control_diagnostics_;

  std::shared_ptr<lcm::LCM> lcm_;
  std::string channel_ = "sim_state";
  std::string control_diagnostics_channel_ = "sim_control_diagnostics";
  // Full control diagnostics are opt-in at the simulator's 500 Hz loop.
  // ENGINEAI_SIM_CONTROL_DIAGNOSTICS_HZ=500 restores per-step publishing.
  int64_t control_diagnostics_period_ns_ = 10'000'000;
  std::chrono::steady_clock::time_point next_control_diagnostics_time_{};
};

#endif  // LCM_INTERFACE_INCLUDE_LCM_INTERFACE_LCM_PUBLISHER_H_

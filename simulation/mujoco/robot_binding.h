#pragma once

#include <mujoco/mujoco.h>
#include <memory>
#include <string>
#include <vector>

#include "lcm_interface/lcm_data_store.h"

// A controller keeps the original single-robot protocol. Only this adapter knows
// where that robot lives in the shared MuJoCo arrays.
class RobotBinding {
 public:
  RobotBinding(std::string prefix, std::shared_ptr<LcmDataStore> store,
               std::vector<std::string> joint_names, std::vector<std::string> contact_frames);

  // Rebuild after every model load; never retain addresses from an old model.
  void Resolve(const mjModel* model);
  void ReadState(const mjData* data) const;
  void ApplyCommand(const mjModel* model, mjData* data) const;
  void ClearCommand() const;

 private:
  int SensorAddress(const mjModel* model, const std::string& name, int dimension,
                    bool required = true) const;

  std::string prefix_;
  std::shared_ptr<LcmDataStore> store_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> contact_frames_;
  std::vector<int> qpos_addresses_;
  std::vector<int> dof_addresses_;
  std::vector<int> actuator_ids_;
  std::vector<int> state_sensor_addresses_;
  std::vector<int> contact_force_addresses_;
  std::vector<int> contact_torque_addresses_;
  const mjModel* bound_model_ = nullptr;
};

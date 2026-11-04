#include "robot_binding.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace {
constexpr const char* kStateSensors[] = {
    "base_link_quaternion", "base_link_position", "base_link_linear_velocity", "base_link_angular_velocity",
    "imu_link_quaternion", "imu_link_position", "imu_link_linear_velocity", "imu_link_angular_velocity",
    "imu_quaternion", "imu_linear_acceleration", "imu_angular_velocity"};
constexpr int kStateDimensions[] = {4, 3, 3, 3, 4, 3, 3, 3, 4, 3, 3};

Eigen::Quaterniond QuaternionAt(const mjData* data, int address) {
  return {data->sensordata[address], data->sensordata[address + 1],
          data->sensordata[address + 2], data->sensordata[address + 3]};
}

Eigen::Vector3d VectorAt(const mjData* data, int address) {
  return Eigen::Map<const Eigen::Vector3d>(data->sensordata + address);
}
}  // namespace

RobotBinding::RobotBinding(std::string prefix, std::shared_ptr<LcmDataStore> store,
                           std::vector<std::string> joint_names, std::vector<std::string> contact_frames)
    : prefix_(std::move(prefix)), store_(std::move(store)), joint_names_(std::move(joint_names)),
      contact_frames_(std::move(contact_frames)) {}

int RobotBinding::SensorAddress(const mjModel* model, const std::string& name, int dimension,
                               bool required) const {
  const int id = mj_name2id(model, mjOBJ_SENSOR, (prefix_ + name).c_str());
  if (id < 0 && !required) return -1;
  if (id < 0 || model->sensor_dim[id] != dimension) {
    throw std::runtime_error("Missing or incompatible sensor: " + prefix_ + name);
  }
  return model->sensor_adr[id];
}

void RobotBinding::Resolve(const mjModel* model) {
  qpos_addresses_.clear();
  dof_addresses_.clear();
  actuator_ids_.clear();
  state_sensor_addresses_.clear();
  contact_force_addresses_.clear();
  contact_torque_addresses_.clear();
  for (const auto& name : joint_names_) {
    const int joint = mj_name2id(model, mjOBJ_JOINT, (prefix_ + name).c_str());
    if (joint < 0 || (model->jnt_type[joint] != mjJNT_HINGE && model->jnt_type[joint] != mjJNT_SLIDE)) {
      throw std::runtime_error("Missing or non-scalar joint: " + prefix_ + name);
    }
    int actuator = -1;
    for (int id = 0; id < model->nu; ++id) {
      if (model->actuator_trntype[id] == mjTRN_JOINT && model->actuator_trnid[2 * id] == joint) {
        if (actuator >= 0) throw std::runtime_error("Multiple actuators for: " + prefix_ + name);
        actuator = id;
      }
    }
    if (actuator < 0 || model->actuator_gear[6 * actuator] != 1.0) {
      throw std::runtime_error("Expected a unit-gear joint actuator for: " + prefix_ + name);
    }
    qpos_addresses_.push_back(model->jnt_qposadr[joint]);
    dof_addresses_.push_back(model->jnt_dofadr[joint]);
    actuator_ids_.push_back(actuator);
  }
  if (static_cast<int>(joint_names_.size()) != store_->num_joints ||
      static_cast<int>(contact_frames_.size()) != store_->num_contacts) {
    throw std::runtime_error("Robot binding does not match controller dimensions: " + prefix_);
  }
  for (size_t i = 0; i < std::size(kStateSensors); ++i) {
    state_sensor_addresses_.push_back(SensorAddress(model, kStateSensors[i], kStateDimensions[i]));
  }
  for (const auto& frame : contact_frames_) {
    // The shipped model measures feet only. Uninstrumented hand contacts must
    // be zero, rather than reading past this robot's sensor array into its peer.
    const std::string suffix = frame == "LINK_FOOT_L" ? "left_foot" :
                               frame == "LINK_FOOT_R" ? "right_foot" : "";
    contact_force_addresses_.push_back(suffix.empty() ? -1 : SensorAddress(model, "force_" + suffix, 3));
    contact_torque_addresses_.push_back(suffix.empty() ? -1 : SensorAddress(model, "torque_" + suffix, 3));
  }
  ClearCommand();
}

void RobotBinding::ReadState(const mjData* data) const {
  SimState state(store_->num_joints, store_->num_contacts, store_->num_single_contact_dimensions);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state.q[i] = data->qpos[qpos_addresses_[i]];
    state.qd[i] = data->qvel[dof_addresses_[i]];
    state.tau[i] = data->actuator_force[actuator_ids_[i]];
  }
  const auto& s = state_sensor_addresses_;
  state.base_link_quaternion = QuaternionAt(data, s[0]);
  state.base_link_position = VectorAt(data, s[1]);
  state.base_link_linear_velocity = VectorAt(data, s[2]);
  state.base_link_angular_velocity = VectorAt(data, s[3]);
  state.imu_link_quaternion = QuaternionAt(data, s[4]);
  state.imu_link_position = VectorAt(data, s[5]);
  state.imu_link_linear_velocity = VectorAt(data, s[6]);
  state.imu_link_angular_velocity = VectorAt(data, s[7]);
  state.imu_sensor_quaternion = QuaternionAt(data, s[8]);
  state.imu_sensor_linear_acceleration = VectorAt(data, s[9]);
  state.imu_sensor_angular_velocity = VectorAt(data, s[10]);
  for (size_t i = 0; i < contact_frames_.size(); ++i) {
    auto& contact = state.contact_force[i];
    if (contact_force_addresses_[i] >= 0 && contact.size() >= 3) {
      contact.head<3>() = -VectorAt(data, contact_force_addresses_[i]);
    }
    if (contact_torque_addresses_[i] >= 0 && contact.size() >= 6) {
      contact.segment<3>(3) = -VectorAt(data, contact_torque_addresses_[i]);
    }
  }
  store_->sim_state.Set(std::move(state));
}

void RobotBinding::ApplyCommand(mjData* data) const {
  const auto command = store_->sim_command.Get();
  const int n = store_->num_joints;
  bool valid = command->q.size() == n && command->qd.size() == n && command->kp.size() == n &&
               command->kd.size() == n && command->tau_ff.size() == n && command->q.allFinite() &&
               command->qd.allFinite() && command->kp.allFinite() && command->kd.allFinite() &&
               command->tau_ff.allFinite();
  const auto received = store_->last_command_time_ns.load();
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const auto age = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() -
                   received;
  valid = valid && received > 0 && age >= 0 && age < 200000000;
  for (int i = 0; i < n; ++i) {
    data->ctrl[actuator_ids_[i]] = valid ? command->kp[i] * (command->q[i] - data->qpos[qpos_addresses_[i]]) +
                                            command->kd[i] * (command->qd[i] - data->qvel[dof_addresses_[i]]) +
                                            command->tau_ff[i] : 0.0;
  }
}

void RobotBinding::ClearCommand() const {
  store_->sim_command.Set(SimCommand(store_->num_joints));
  store_->last_command_time_ns.store(0);
}

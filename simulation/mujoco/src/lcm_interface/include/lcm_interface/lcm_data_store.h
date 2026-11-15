#ifndef LCM_INTERFACE_INCLUDE_LCM_INTERFACE_LCM_DATA_STORE_H_
#define LCM_INTERFACE_INCLUDE_LCM_INTERFACE_LCM_DATA_STORE_H_

#include <Eigen/Dense>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <vector>

#include "guarded_data/guarded_data.h"

namespace lcm_interface {

inline int64_t SimControlDiagnosticsPeriodNsFromEnvironment() {
  constexpr int64_t kDefaultPeriodNs = 10'000'000;  // 100 Hz
  const char* value = std::getenv("ENGINEAI_SIM_CONTROL_DIAGNOSTICS_HZ");
  if (value == nullptr || *value == '\0') return kDefaultPeriodNs;

  char* end = nullptr;
  const double hz = std::strtod(value, &end);
  if (end == value || *end != '\0' || !std::isfinite(hz) || hz <= 0.0) {
    return kDefaultPeriodNs;
  }
  const double period_ns = 1'000'000'000.0 / hz;
  if (period_ns < 1.0) return 1;
  if (period_ns > static_cast<double>(std::numeric_limits<int64_t>::max())) {
    return kDefaultPeriodNs;
  }
  return static_cast<int64_t>(period_ns);
}

}  // namespace lcm_interface

class SimCommand {
 public:
  SimCommand() = default;
  SimCommand(int num_joints) { Reset(num_joints); }
  ~SimCommand() = default;

  void Reset(int num_joints) {
    q = Eigen::VectorXd::Zero(num_joints);
    qd = Eigen::VectorXd::Zero(num_joints);
    tau_ff = Eigen::VectorXd::Zero(num_joints);
    kp = Eigen::VectorXd::Zero(num_joints);
    kd = Eigen::VectorXd::Zero(num_joints);
  }

  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd tau_ff;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
};

class SimState {
 public:
  SimState() = default;
  SimState(int num_joints, int num_contacts, int num_single_contact_dimensions) {
    Reset(num_joints, num_contacts, num_single_contact_dimensions);
  }
  ~SimState() = default;

  void Reset(int num_joints, int num_contacts, int num_single_contact_dimensions) {
    sim_time = 0.0;
    q = Eigen::VectorXd::Zero(num_joints);
    qd = Eigen::VectorXd::Zero(num_joints);
    tau = Eigen::VectorXd::Zero(num_joints);
    contact_force = std::vector<Eigen::VectorXd>(num_contacts, Eigen::VectorXd::Zero(num_single_contact_dimensions));

    imu_sensor_quaternion = Eigen::Quaterniond::Identity();
    imu_sensor_linear_acceleration = Eigen::Vector3d::Zero();
    imu_sensor_angular_velocity = Eigen::Vector3d::Zero();

    imu_link_quaternion = Eigen::Quaterniond::Identity();
    imu_link_position = Eigen::Vector3d::Zero();
    imu_link_linear_velocity = Eigen::Vector3d::Zero();
    imu_link_angular_velocity = Eigen::Vector3d::Zero();

    base_link_quaternion = Eigen::Quaterniond::Identity();
    base_link_position = Eigen::Vector3d::Zero();
    base_link_linear_velocity = Eigen::Vector3d::Zero();
    base_link_angular_velocity = Eigen::Vector3d::Zero();
  }

  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd tau;
  // MuJoCo time captured in the same guarded snapshot as the state vectors.
  double sim_time = 0.0;

  Eigen::Quaterniond imu_sensor_quaternion;
  Eigen::Vector3d imu_sensor_linear_acceleration;
  Eigen::Vector3d imu_sensor_angular_velocity;

  Eigen::Quaterniond imu_link_quaternion;
  Eigen::Vector3d imu_link_position;
  Eigen::Vector3d imu_link_linear_velocity;
  Eigen::Vector3d imu_link_angular_velocity;

  Eigen::Quaterniond base_link_quaternion;
  Eigen::Vector3d base_link_position;
  Eigen::Vector3d base_link_linear_velocity;
  Eigen::Vector3d base_link_angular_velocity;

  std::vector<Eigen::VectorXd> contact_force;
};

class SimControlDiagnostics {
 public:
  SimControlDiagnostics() = default;
  explicit SimControlDiagnostics(int num_joints) { Reset(num_joints); }

  void Reset(int num_joints) {
    nominal_ctrl = Eigen::VectorXd::Zero(num_joints);
    effective_ctrl = Eigen::VectorXd::Zero(num_joints);
    injected_noise = Eigen::VectorXd::Zero(num_joints);
    sim_time = 0.0;
    host_time_ns = 0;
    command_age_ms = -1.0;
    command_valid = false;
  }

  Eigen::VectorXd nominal_ctrl;
  Eigen::VectorXd effective_ctrl;
  Eigen::VectorXd injected_noise;
  double sim_time = 0.0;
  int64_t host_time_ns = 0;
  double command_age_ms = -1.0;
  bool command_valid = false;
};

class LcmDataStore {
 public:
  LcmDataStore(int num_joints, int num_contacts, int num_single_contact_dimensions)
      : num_joints(num_joints),
        num_contacts(num_contacts),
        num_single_contact_dimensions(num_single_contact_dimensions),
        control_diagnostics_period_ns_(lcm_interface::SimControlDiagnosticsPeriodNsFromEnvironment()) {
    sim_command.Set(SimCommand(num_joints));
    sim_state.Set(SimState(num_joints, num_contacts, num_single_contact_dimensions));
    sim_control_diagnostics.Set(SimControlDiagnostics(num_joints));
  }

  bool ShouldCaptureControlDiagnostics(int64_t now_ns) {
    int64_t next_ns = next_control_diagnostics_time_ns.load(std::memory_order_relaxed);
    while (now_ns >= next_ns) {
      const int64_t candidate_ns = now_ns + control_diagnostics_period_ns_;
      if (next_control_diagnostics_time_ns.compare_exchange_weak(
              next_ns, candidate_ns, std::memory_order_relaxed, std::memory_order_relaxed)) {
        return true;
      }
    }
    return false;
  }

  int64_t control_diagnostics_period_ns() const { return control_diagnostics_period_ns_; }

  data::GuardedData<SimCommand> sim_command;
  data::GuardedData<SimState> sim_state;
  data::GuardedData<SimControlDiagnostics> sim_control_diagnostics;
  std::atomic<int64_t> last_command_time_ns{0};

  int num_joints = 0;
  int num_contacts = 0;
  int num_single_contact_dimensions = 0;

 private:
  const int64_t control_diagnostics_period_ns_;
  std::atomic<int64_t> next_control_diagnostics_time_ns{0};
};

#endif  // LCM_INTERFACE_INCLUDE_LCM_INTERFACE_LCM_DATA_STORE_H_

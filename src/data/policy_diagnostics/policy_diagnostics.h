#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace data {

// One control-step snapshot of the policy pipeline. Values are kept in the
// runner's deployment joint order and are published identically in sim and on
// the robot through VariantStore/ROS2.
struct PolicyDiagnostics {
  int64_t source_monotonic_ns = 0;
  std::string source_param_tag;
  bool active = false;
  bool policy_output_valid = false;
  bool policy_recurrent = false;
  bool entry_transition_active = false;
  double entry_transition_elapsed = 0.0;
  double entry_transition_duration = 0.0;

  std::vector<double> transformed_base_angular_velocity;
  std::vector<double> projected_gravity;
  std::vector<double> policy_command;
  std::vector<double> observation_single;
  std::vector<double> observation;
  std::vector<double> raw_action;
  std::vector<double> clipped_action;
  std::vector<double> controlled_action;
  std::vector<double> q_des_policy;
  std::vector<double> q_des_command;
  std::vector<double> qd_des_policy;
  std::vector<double> qd_des_command;
  std::vector<double> feed_forward_torque;
  std::vector<double> stiffness;
  std::vector<double> damping;
};

}  // namespace data

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "basic_param/basic_param.h"
#include "parameter/parameter_loader.h"

namespace data {

class RlSupermimicParam : public BasicParam {
 public:
  explicit RlSupermimicParam(std::string_view tag = "rl_supermimic_come_here") : BasicParam(tag) {}

  DEFINE_PARAM_SCOPE(scope_);
  int LOAD_PARAM(schema_version);
  std::string LOAD_PARAM(method);
  std::string LOAD_PARAM(robot);
  std::string LOAD_PARAM(policy_file);
  std::string LOAD_PARAM(reference_file);
  double LOAD_PARAM(control_dt);
  double LOAD_PARAM(physics_dt);
  double LOAD_PARAM(hold_seconds);
  int LOAD_PARAM(history_length);
  int LOAD_PARAM(observation_dim);
  int LOAD_PARAM(goal_dim);
  int LOAD_PARAM(input_dim);
  int LOAD_PARAM(output_dim);
  std::string LOAD_PARAM(history_order);
  std::string LOAD_PARAM(deployment_precision);
  std::vector<std::string> LOAD_PARAM(joint_names);
  std::vector<std::string> LOAD_PARAM(mujoco_joint_names);
  Eigen::VectorXd LOAD_PARAM(default_joint_pos);
  Eigen::VectorXd LOAD_PARAM(action_scale);
  Eigen::VectorXd LOAD_PARAM(joint_stiffness);
  Eigen::VectorXd LOAD_PARAM(joint_damping);
  Eigen::VectorXd LOAD_PARAM(effort_limits);
  std::optional<Eigen::Vector3d> LOAD_PARAM(imu_install_bias);
  std::optional<bool> LOAD_PARAM(entry_transition_enabled);
  std::optional<double> LOAD_PARAM(entry_transition_duration);
  std::optional<double> LOAD_PARAM(entry_transition_min_duration);
  std::optional<double> LOAD_PARAM(entry_transition_max_duration);
  std::optional<double> LOAD_PARAM(entry_transition_max_joint_velocity);
  std::optional<double> LOAD_PARAM(entry_transition_max_joint_acceleration);
  std::optional<double> LOAD_PARAM(entry_transition_source_tracking_error);
};

}  // namespace data

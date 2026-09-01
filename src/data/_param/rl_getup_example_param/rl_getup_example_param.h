#pragma once

#include <optional>
#include <string>
#include <vector>

#include "basic_param/basic_param.h"
#include "parameter/parameter_loader.h"

namespace data {

class RlGetupExampleParam : public BasicParam {
 public:
  RlGetupExampleParam(std::string_view tag = "rl_getup_example") : BasicParam(tag) {}

  DEFINE_PARAM_SCOPE(scope_);

  std::string LOAD_PARAM(policy_file);

  int LOAD_PARAM(num_one_step_observations);
  int LOAD_PARAM(num_observations);
  int LOAD_PARAM(num_include_obs_steps);
  int LOAD_PARAM(num_actions);

  std::vector<std::string> LOAD_PARAM(joint_names);
  Eigen::VectorXd LOAD_PARAM(default_joint_pos);
  Eigen::VectorXd LOAD_PARAM(joint_stiffness);
  Eigen::VectorXd LOAD_PARAM(joint_damping);
  Eigen::VectorXd LOAD_PARAM(action_scale);
  double LOAD_PARAM(action_rescale);
  double LOAD_PARAM(action_clip);

  double LOAD_PARAM(observation_scale_angular_vel);
  double LOAD_PARAM(observation_scale_dof_pos);
  double LOAD_PARAM(observation_scale_dof_vel);
  double LOAD_PARAM(observation_clip);

  std::string LOAD_PARAM(first_frame_history_mode);
  float LOAD_PARAM(control_dt);
  std::optional<Eigen::Vector3d> LOAD_PARAM(imu_install_bias);
  double LOAD_PARAM(entry_max_upright_projected_gravity_z);
  double LOAD_PARAM(entry_max_angular_velocity_norm);
  double LOAD_PARAM(success_upright_projected_gravity_z);
  double LOAD_PARAM(success_max_angular_velocity_norm);
  double LOAD_PARAM(success_hold_duration);
  double LOAD_PARAM(timeout_duration);

  void Update() {
    LOAD_PARAM(policy_file);
    LOAD_PARAM(joint_stiffness);
    LOAD_PARAM(joint_damping);
    LOAD_PARAM(action_scale);
    LOAD_PARAM(action_rescale);
    LOAD_PARAM(control_dt);
    LOAD_PARAM(entry_max_upright_projected_gravity_z);
    LOAD_PARAM(entry_max_angular_velocity_norm);
    LOAD_PARAM(success_upright_projected_gravity_z);
    LOAD_PARAM(success_max_angular_velocity_norm);
    LOAD_PARAM(success_hold_duration);
    LOAD_PARAM(timeout_duration);
  }
};

}  // namespace data

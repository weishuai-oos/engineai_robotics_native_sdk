#pragma once

#include <iostream>
#include <optional>
#include <string>

#include "basic_param/basic_param.h"
#include "parameter/parameter_loader.h"

namespace data {

class RlDanceExampleParam : public BasicParam {
 public:
  RlDanceExampleParam(std::string_view tag = "rl_dance_example") : BasicParam(tag) { num_actions = joint_names.size(); }

  DEFINE_PARAM_SCOPE(scope_);

  std::string LOAD_PARAM(policy_file);
  std::string LOAD_PARAM(trajectory_file_npz);

  std::vector<std::string> LOAD_PARAM(joint_names);
  Eigen::VectorXd LOAD_PARAM(joint_stiffness);
  Eigen::VectorXd LOAD_PARAM(joint_damping);
  Eigen::VectorXd LOAD_PARAM(default_joint_pos);

  std::vector<std::string> LOAD_PARAM(observation_names);
  std::vector<int> LOAD_PARAM(observation_history_lengths);
  Eigen::VectorXd LOAD_PARAM(action_scale);
  bool LOAD_PARAM(resident_control);
  std::optional<double> LOAD_PARAM(startup_interpolation_duration);
  std::optional<double> LOAD_PARAM(lower_body_startup_interpolation_duration);
  std::optional<double> LOAD_PARAM(upper_body_startup_interpolation_duration);
  std::optional<double> LOAD_PARAM(lower_body_policy_blend_duration);
  std::optional<bool> LOAD_PARAM(entry_transition_enabled);
  std::optional<double> LOAD_PARAM(entry_transition_duration);
  std::optional<double> LOAD_PARAM(entry_transition_min_duration);
  std::optional<double> LOAD_PARAM(entry_transition_max_duration);
  std::optional<double> LOAD_PARAM(entry_transition_max_joint_velocity);
  std::optional<double> LOAD_PARAM(entry_transition_max_joint_acceleration);
  std::optional<double> LOAD_PARAM(entry_transition_reference_pose_weight);
  std::optional<double> LOAD_PARAM(entry_transition_source_tracking_error);
  std::optional<int> LOAD_PARAM(trajectory_body_index);
  std::optional<std::string> LOAD_PARAM(trajectory_end_behavior);

  int num_actions;
};

}  // namespace data

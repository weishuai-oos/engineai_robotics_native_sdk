#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "basic_param/basic_param.h"
#include "parameter/parameter_loader.h"

namespace data {

class RlWalkingLeolabExampleParam : public BasicParam {
 public:
  RlWalkingLeolabExampleParam(std::string_view tag = "rl_walking_leolab_example") : BasicParam(tag) {}

  DEFINE_PARAM_SCOPE(scope_);

  std::string LOAD_PARAM(policy_file);
  bool LOAD_PARAM(policy_recurrent);
  int LOAD_PARAM(num_one_step_observations);
  int LOAD_PARAM(num_observations);
  int LOAD_PARAM(num_actions);
  int LOAD_PARAM(num_include_obs_steps);

  std::string LOAD_PARAM(observation_input_name);
  std::string LOAD_PARAM(hidden_input_name);
  std::string LOAD_PARAM(cell_input_name);
  std::string LOAD_PARAM(action_output_name);
  std::string LOAD_PARAM(hidden_output_name);
  std::string LOAD_PARAM(cell_output_name);

  std::vector<std::string> LOAD_PARAM(policy_joint_names);
  std::vector<std::string> LOAD_PARAM(joint_names);
  std::vector<std::string> LOAD_PARAM(override_action_joint_names);
  double LOAD_PARAM(override_action_value);

  Eigen::VectorXd LOAD_PARAM(default_joint_pos);
  Eigen::VectorXd LOAD_PARAM(joint_stiffness);
  Eigen::VectorXd LOAD_PARAM(joint_damping);
  Eigen::VectorXd LOAD_PARAM(action_scale);

  double LOAD_PARAM(action_clip);
  double LOAD_PARAM(observation_clip);
  double LOAD_PARAM(control_dt);

  std::optional<bool> LOAD_PARAM(entry_transition_enabled);
  std::optional<double> LOAD_PARAM(entry_transition_duration);
  std::optional<double> LOAD_PARAM(entry_transition_min_duration);
  std::optional<double> LOAD_PARAM(entry_transition_max_duration);
  std::optional<double> LOAD_PARAM(entry_transition_max_joint_velocity);
  std::optional<double> LOAD_PARAM(entry_transition_max_joint_acceleration);
  std::optional<double> LOAD_PARAM(entry_transition_reference_pose_weight);
  std::optional<double> LOAD_PARAM(entry_transition_source_tracking_error);

  bool LOAD_PARAM(enable_remote_command_lpf);
  double LOAD_PARAM(remote_command_sampling_frequency);
  double LOAD_PARAM(remote_command_cut_off_frequency);
  double LOAD_PARAM(remote_command_activation_threshold);
  double LOAD_PARAM(remote_command_release_threshold);
  double LOAD_PARAM(remote_command_translation_axis_switch_margin);
  double LOAD_PARAM(remote_command_reversal_pause_sec);
  double LOAD_PARAM(remote_command_tactical_front_offset_deg);
  Eigen::Vector3d LOAD_PARAM(command_scale_pos);
  Eigen::Vector3d LOAD_PARAM(command_scale_neg);
  std::optional<Eigen::Vector3d> LOAD_PARAM(imu_install_bias);

  void Update() {
    LOAD_PARAM(default_joint_pos);
    LOAD_PARAM(joint_stiffness);
    LOAD_PARAM(joint_damping);
    LOAD_PARAM(action_scale);
    LOAD_PARAM(entry_transition_enabled);
    LOAD_PARAM(entry_transition_duration);
    LOAD_PARAM(entry_transition_min_duration);
    LOAD_PARAM(entry_transition_max_duration);
    LOAD_PARAM(entry_transition_max_joint_velocity);
    LOAD_PARAM(entry_transition_max_joint_acceleration);
    LOAD_PARAM(entry_transition_reference_pose_weight);
    LOAD_PARAM(entry_transition_source_tracking_error);
    LOAD_PARAM(enable_remote_command_lpf);
    LOAD_PARAM(remote_command_cut_off_frequency);
    LOAD_PARAM(remote_command_activation_threshold);
    LOAD_PARAM(remote_command_release_threshold);
    LOAD_PARAM(remote_command_translation_axis_switch_margin);
    LOAD_PARAM(remote_command_reversal_pause_sec);
    LOAD_PARAM(remote_command_tactical_front_offset_deg);
    LOAD_PARAM(command_scale_pos);
    LOAD_PARAM(command_scale_neg);
  }
};

}  // namespace data

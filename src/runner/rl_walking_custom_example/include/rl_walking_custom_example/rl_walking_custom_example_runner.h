#pragma once

#include "basic/motion_runner.h"
#include "basic/runner_registry.h"
#include "math/first_order_low_pass_filter.h"
#include "math/mnn_model.h"
#include "motion_transition/entry_command_transition.h"
#include "parameter/global_config_initializer.h"
#include "rl_walking_custom_example_param/rl_walking_custom_example_param.h"
#include "tool/string_join.h"

namespace runner {

class RlWalkingCustomExampleRunner : public MotionRunner {
 public:
  RlWalkingCustomExampleRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store);
  ~RlWalkingCustomExampleRunner() = default;

  bool Enter() override;
  void Run() override;
  TransitionState TryExit() override;
  bool Exit() override;
  void End() override;
  void SetupContext() override;
  void TeardownContext() override;

  void Log();

 private:
  void UpdateRemoteCommand();
  void CalculateObservation();
  void CalculateMotorCommand();
  bool InitializeEntryTransition();
  void ApplyEntryTransition();
  bool InitializeUpperBodyLock();
  void ApplyUpperBodyLock();
  void SendMotorCommand();

  std::shared_ptr<data::RlWalkingCustomExampleParam> param_;
  std::string last_param_tag_ = "";
  float time_ = 0.0;
  bool is_first_time_ = true;

  std::unique_ptr<math::MNNModel> mlp_net_;
  Eigen::MatrixXd mlp_net_observation_;
  Eigen::VectorXd mlp_net_action_;

  Eigen::VectorXd q_real_;
  Eigen::VectorXd qd_real_;
  Eigen::VectorXd q_des_;
  Eigen::VectorXd qd_des_;
  Eigen::VectorXd tau_ff_des_;
  Eigen::VectorXi active_joint_idx_;

  bool upper_body_lock_enabled_ = false;
  bool upper_body_lock_initialized_ = false;
  Eigen::VectorXi upper_body_lock_joint_idx_;
  Eigen::VectorXi upper_body_lock_action_idx_;
  Eigen::VectorXd upper_body_lock_target_q_;

  Eigen::VectorXd default_joint_q_;
  Eigen::VectorXd joint_kp_;
  Eigen::VectorXd joint_kd_;
  Eigen::VectorXd joint_kp_cmd_;
  Eigen::VectorXd joint_kd_cmd_;
  Eigen::VectorXd action_scale_;
  Eigen::VectorXd entry_reference_q_;
  motion_transition::EntryCommandTransition entry_transition_;

  Eigen::Vector3d imu_install_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d command_ = Eigen::Vector3d::Zero();
  std::unique_ptr<math::FirstOrderLowPassFilter<Eigen::Vector3d>> lpf_command_;
};
}  // namespace runner

REGISTER_RUNNER(RlWalkingCustomExampleRunner, "rl_walking_custom_example_runner", kMotion)

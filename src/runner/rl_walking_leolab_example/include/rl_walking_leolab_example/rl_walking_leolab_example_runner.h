#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "basic/motion_runner.h"
#include "basic/runner_registry.h"
#include "math/first_order_low_pass_filter.h"
#include "math/mnn_model.h"
#include "motion_transition/entry_command_transition.h"
#include "parameter/global_config_initializer.h"
#include "rl_walking_leolab_example/fixed_remote_command_shaper.h"
#include "rl_walking_leolab_example/mnn_recurrent_model.h"
#include "rl_walking_leolab_example_param/rl_walking_leolab_example_param.h"

namespace runner {

class RlWalkingLeolabExampleRunner : public MotionRunner {
 public:
  RlWalkingLeolabExampleRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store);
  ~RlWalkingLeolabExampleRunner() = default;

  bool Enter() override;
  void Run() override;
  TransitionState TryExit() override;
  bool Exit() override;
  void End() override;
  void SetupContext() override;
  void TeardownContext() override;

 private:
  bool ValidateParam() const;
  bool ValidatePolicyContract();
  bool BuildJointMapping();
  bool BuildOverrideActionIndices();
  bool ComputeBaseState(Eigen::Vector3d* base_ang_vel, Eigen::Vector3d* projected_gravity) const;
  void UpdateRemoteCommand();
  void CalculateObservation();
  void CalculateMotorCommand();
  bool InitializeEntryTransition();
  void ApplyEntryTransition();
  void HoldCurrentPose();
  void SendMotorCommand();

  std::shared_ptr<data::RlWalkingLeolabExampleParam> param_;
  std::string last_param_tag_;
  double time_ = 0.0;
  bool is_first_time_ = true;
  bool policy_output_valid_ = true;

  std::unique_ptr<math::MNNModel> mlp_net_;
  std::unique_ptr<math::MNNRecurrentModel> recurrent_net_;
  Eigen::MatrixXd observation_history_;
  Eigen::VectorXd policy_observation_;
  Eigen::VectorXd mlp_net_action_;
  Eigen::VectorXf recurrent_hidden_;
  Eigen::VectorXf recurrent_cell_;

  Eigen::VectorXd q_real_;
  Eigen::VectorXd qd_real_;
  Eigen::VectorXd q_des_;
  Eigen::VectorXd qd_des_;
  Eigen::VectorXd tau_ff_des_;
  Eigen::VectorXi policy2deploy_joint_idx_;
  Eigen::VectorXi override_action_idx_;

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
  FixedRemoteCommandShaper remote_command_shaper_;
  std::unique_ptr<math::FirstOrderLowPassFilter<Eigen::Vector3d>> lpf_command_;
};

}  // namespace runner

REGISTER_RUNNER(RlWalkingLeolabExampleRunner, "rl_walking_leolab_example_runner", kMotion)

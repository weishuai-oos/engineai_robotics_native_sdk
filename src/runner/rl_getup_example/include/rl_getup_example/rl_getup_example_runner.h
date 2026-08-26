#pragma once

#include <vector>

#include "basic/motion_runner.h"
#include "basic/runner_registry.h"
#include "math/mnn_model.h"
#include "parameter/global_config_initializer.h"
#include "rl_getup_example/host_policy_contract.h"
#include "rl_getup_example_param/rl_getup_example_param.h"

namespace runner {

class RlGetupExampleRunner : public MotionRunner {
 public:
  RlGetupExampleRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store);
  ~RlGetupExampleRunner() = default;

  bool Enter() override;
  void Run() override;
  TransitionState TryExit() override;
  bool IsTransitionAllowed(std::string_view target_motion) const override;
  bool Exit() override;
  void End() override;
  void SetupContext() override;
  void TeardownContext() override;

 private:
  bool ValidateParam() const;
  bool ValidatePolicyContract();
  bool BuildJointMapping();
  bool ComputeBaseState(Eigen::Vector3d* base_ang_vel, Eigen::Vector3d* projected_gravity) const;
  bool ValidateEntryState();
  void CalculateObservation();
  void CalculateMotorCommand();
  void ClampAndCheckTargets();
  void UpdateCompletionState();
  void HoldCurrentPose();
  void SendMotorCommand();

  std::shared_ptr<data::RlGetupExampleParam> param_;
  std::string last_param_tag_;
  float time_ = 0.0;
  bool is_first_time_ = true;

  std::unique_ptr<math::MNNModel> mlp_net_;
  std::vector<double> observation_history_;
  Eigen::VectorXd policy_observation_;
  Eigen::VectorXd mlp_net_action_;

  Eigen::VectorXd q_real_;
  Eigen::VectorXd qd_real_;
  Eigen::VectorXd q_des_;
  Eigen::VectorXd qd_des_;
  Eigen::VectorXd tau_ff_des_;
  Eigen::VectorXi policy2deploy_joint_idx_;

  Eigen::VectorXd joint_kp_;
  Eigen::VectorXd joint_kd_;
  Eigen::VectorXd action_scale_;
  Eigen::Vector3d imu_install_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_projected_gravity_ = Eigen::Vector3d::Zero();
  double last_base_ang_vel_norm_ = 0.0;
  double success_hold_time_ = 0.0;
  bool completed_by_posture_ = false;
  bool timed_out_ = false;
  bool exit_status_reported_ = false;
  bool observation_valid_ = true;
};

}  // namespace runner

REGISTER_RUNNER(RlGetupExampleRunner, "rl_getup_example_runner", kMotion)

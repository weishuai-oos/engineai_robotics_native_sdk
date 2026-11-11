#pragma once

#include "basic/motion_runner.h"
#include "basic/runner_registry.h"
#include "motion_transition/entry_command_transition.h"
#include "motion_transition/reference_pose_gate.h"
#include "rl_supermimic/flat_policy.h"
#include "rl_supermimic/policy_contract.h"
#include "rl_supermimic/reference.h"
#include "rl_supermimic_param/rl_supermimic_param.h"

namespace runner {

class RlSupermimicRunner : public MotionRunner {
 public:
  RlSupermimicRunner(std::string_view name, const std::shared_ptr<data::DataStore>& store)
      : MotionRunner(name, store) {}
  bool Enter() override;
  void Run() override;
  TransitionState TryExit() override { return TransitionState::kCompleted; }
  bool Exit() override { return true; }
  void End() override {}
  void SetupContext() override { data_store_->parallel_by_classic_parser.store(false); }
  void TeardownContext() override {}

 private:
  void ValidateParameters();
  void BuildJointMapping();
  void ReadState(Eigen::Matrix3d* rotation, Eigen::Vector3d* body_angular_velocity);
  void ResetPolicy(const Eigen::Matrix3d& rotation);
  void RunReferencePoseEntry();
  void Fail(const std::string& reason);
  void SendCommand();

  std::shared_ptr<data::RlSupermimicParam> param_;
  std::unique_ptr<supermimic::FlatPolicy> policy_;
  supermimic::Reference reference_;
  supermimic::Sequence sequence_;
  supermimic::History history_;
  Eigen::VectorXd previous_action_;
  Eigen::VectorXi mapping_;
  Eigen::VectorXd actual_q_, actual_qd_;
  Eigen::Matrix3d installation_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d yaw_alignment_ = Eigen::Matrix3d::Identity();
  motion_transition::JointCommand command_;
  motion_transition::JointCommand live_command_;
  motion_transition::JointCommand reference_pose_command_;
  motion_transition::EntryCommandTransition transition_;
  ReferencePoseGate reference_pose_gate_;
  bool entry_transition_to_reference_pose_ = false;
  bool policy_start_pending_ = false;
  double reference_pose_log_elapsed_ = 0.0;
  bool reference_pose_log_pending_ = true;
  bool first_observation_ = true;
};

}  // namespace runner

REGISTER_RUNNER(RlSupermimicRunner, "rl_supermimic_runner", kMotion)

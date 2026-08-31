#include "motion_transition/entry_command_transition.h"

#include <gtest/gtest.h>

namespace runner::motion_transition {
namespace {

JointCommand ConstantCommand(int size, double q, double qd, double kp, double kd, double tau_ff) {
  JointCommand command;
  command.q = Eigen::VectorXd::Constant(size, q);
  command.qd = Eigen::VectorXd::Constant(size, qd);
  command.kp = Eigen::VectorXd::Constant(size, kp);
  command.kd = Eigen::VectorXd::Constant(size, kd);
  command.tau_ff = Eigen::VectorXd::Constant(size, tau_ff);
  return command;
}

TEST(EntryCommandTransitionTest, KeepsPolicyRunningWhileCommandStartsContinuously) {
  EntryCommandTransition transition;
  EntryTransitionConfig config;
  config.nominal_duration = 0.20;
  config.min_duration = 0.20;
  config.max_duration = 0.20;
  config.reference_pose_weight = 0.0;
  ASSERT_TRUE(transition.Configure(config));

  const JointCommand source = ConstantCommand(2, 0.0, 0.0, 20.0, 1.0, 1.0);
  const JointCommand target = ConstantCommand(2, 1.0, 0.0, 100.0, 4.0, 0.0);
  const Eigen::VectorXd actual_q = Eigen::VectorXd::Zero(2);
  ASSERT_TRUE(transition.Start(source, actual_q, Eigen::VectorXd::Zero(2), target));

  JointCommand output;
  ASSERT_TRUE(transition.Apply(target, target.q, actual_q, 0.02, &output));
  EXPECT_GT(output.q.minCoeff(), 0.0);
  EXPECT_LT(output.q.maxCoeff(), 0.02);
  EXPECT_GT(output.kp.minCoeff(), 20.0);
  EXPECT_LT(output.kp.maxCoeff(), 100.0);
  EXPECT_TRUE(transition.IsActive());

  for (int i = 1; i < 10; ++i) {
    ASSERT_TRUE(transition.Apply(target, target.q, output.q, 0.02, &output));
  }
  EXPECT_TRUE(transition.IsComplete());
  EXPECT_TRUE(output.q.isApprox(target.q));
  EXPECT_TRUE(output.qd.isApprox(target.qd));
  EXPECT_TRUE(output.kp.isApprox(target.kp));
  EXPECT_TRUE(output.kd.isApprox(target.kd));
  EXPECT_TRUE(output.tau_ff.isApprox(target.tau_ff));
}

TEST(EntryCommandTransitionTest, UsesReferenceOnlyAsDecayingHint) {
  EntryTransitionConfig config;
  config.nominal_duration = 0.20;
  config.min_duration = 0.20;
  config.max_duration = 0.20;
  config.reference_pose_weight = 0.5;

  EntryCommandTransition transition;
  ASSERT_TRUE(transition.Configure(config));
  const JointCommand source = ConstantCommand(1, 0.0, 0.0, 20.0, 1.0, 0.0);
  const JointCommand target = ConstantCommand(1, 1.0, 0.0, 80.0, 3.0, 0.0);
  const Eigen::VectorXd reference = Eigen::VectorXd::Constant(1, 0.25);
  ASSERT_TRUE(transition.Start(source, source.q, source.qd, target));

  JointCommand output;
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(transition.Apply(target, reference, output.q.size() == 1 ? output.q : source.q, 0.02, &output));
  }
  EXPECT_LT(output.q(0), 0.5);

  for (int i = 5; i < 10; ++i) {
    ASSERT_TRUE(transition.Apply(target, reference, output.q, 0.02, &output));
  }
  EXPECT_TRUE(output.q.isApprox(target.q));
}

TEST(EntryCommandTransitionTest, AdaptsDurationToInitialPositionGap) {
  EntryCommandTransition transition;
  EntryTransitionConfig config;
  config.nominal_duration = 0.05;
  config.min_duration = 0.05;
  config.max_duration = 0.50;
  config.max_joint_velocity = 2.0;
  config.max_joint_acceleration = 100.0;
  config.reference_pose_weight = 0.0;
  ASSERT_TRUE(transition.Configure(config));

  const JointCommand source = ConstantCommand(1, 0.0, 0.0, 20.0, 1.0, 0.0);
  const JointCommand target = ConstantCommand(1, 0.4, 0.0, 80.0, 3.0, 0.0);
  ASSERT_TRUE(transition.Start(source, source.q, source.qd, target));

  JointCommand output;
  ASSERT_TRUE(transition.Apply(target, target.q, source.q, 0.01, &output));
  EXPECT_NEAR(transition.duration(), 0.375, 1e-9);
}

}  // namespace
}  // namespace runner::motion_transition

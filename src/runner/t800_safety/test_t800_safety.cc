#include "t800_safety/t800_safety.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>

#include "data/variant_store/variant_store.h"

namespace runner::t800_safety {
namespace {
std::string ContractPath() {
  return (std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path() /
          "assets/config/t800/safety/t800_safety_contract.json").string();
}

TEST(T800JointEnvelopeTest, LoadsV2AndValidatesAllJoints) {
  T800JointEnvelope envelope; std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  EXPECT_EQ(envelope.schema_version(), 2); EXPECT_EQ(envelope.qualification(), "urdf_derived_provisional");
  EXPECT_EQ(envelope.joint(23).sdk_name, "J23_HEAD_PITCH"); EXPECT_EQ(envelope.joint(24).sdk_name, "J24_HEAD_YAW");
}

TEST(T800SafetyGateTest, ClipsLimitsAndSlewAndRejectsFullTorque) {
  T800JointEnvelope envelope; std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  JointCommandSafetyGate gate(envelope); T800JointState measured; measured.q.fill(0); measured.qd.fill(0);
  T800CommandFrame command; command.q_des.fill(0); command.qd_des.fill(0); command.kp.fill(0); command.kd.fill(0); command.tau_ff.fill(0);
  command.q_des[0] = envelope.joint(0).operational_position_upper + 1; command.qd_des[0] = envelope.joint(0).operational_velocity + 1;
  auto first = gate.Evaluate(command, measured, 0.02); ASSERT_TRUE(first.accepted); EXPECT_EQ(first.decision, SafetyDecision::kClipped);
  command.full_torque_enabled = true; auto rejected = gate.Evaluate(command, measured, 0.02); EXPECT_FALSE(rejected.accepted); EXPECT_NE(rejected.reason_mask & kReasonFullTorqueUnsupported, 0U);
}

TEST(HeadJointHealthMonitorTest, SanitizesHeadWithoutHidingFrameFault) {
  T800JointEnvelope envelope; std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  HeadJointHealthMonitor monitor(envelope); T800JointState state; state.q.fill(0); state.qd.fill(0); state.tau.fill(0);
  T800MotorFlags flags; flags.enable.fill(1); auto good = monitor.Update(state, flags); EXPECT_FALSE(good.frame_fault);
  state.q[kHeadPitchIndex] = std::numeric_limits<double>::quiet_NaN(); state.q[0] = std::numeric_limits<double>::quiet_NaN();
  auto bad = monitor.Update(state, flags); EXPECT_EQ(bad.head_fault_mask[kHeadPitchIndex], 1); EXPECT_TRUE(bad.frame_fault); EXPECT_TRUE(std::isfinite(bad.sanitized_state.q[kHeadPitchIndex]));
}

TEST(HeadJointHealthMonitorTest, ClearsMaskAfterRecoveryHysteresis) {
  T800JointEnvelope envelope; std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  HeadJointHealthMonitor monitor(envelope); T800JointState state{}; state.q.fill(0); state.qd.fill(0); state.tau.fill(0);
  T800MotorFlags flags{}; flags.enable.fill(1);
  auto good = monitor.Update(state, flags, 0, true); EXPECT_EQ(good.head_fault_mask[23], 0);
  state.q[23] = std::numeric_limits<double>::quiet_NaN(); auto failed = monitor.Update(state, flags, 0, true); EXPECT_EQ(failed.head_fault_mask[23], 1);
  state.q[23] = 0.1;
  for (int frame = 1; frame < kHeadRecoveryHealthyFrames; ++frame) {
    const auto recovering = monitor.Update(state, flags, 0, true);
    EXPECT_EQ(recovering.health[23], JointHealth::kRecovering);
    EXPECT_EQ(recovering.head_fault_mask[23], 1);
  }
  const auto recovered = monitor.Update(state, flags, 0, true);
  EXPECT_EQ(recovered.health[23], JointHealth::kHealthy);
  EXPECT_EQ(recovered.head_fault_mask[23], 0);
}

TEST(HeadJointHealthMonitorTest, RejectsFiniteImplausibleHeadPositionJump) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  HeadJointHealthMonitor monitor(envelope);
  T800JointState state{};
  state.q.fill(0.0);
  state.qd.fill(0.0);
  T800MotorFlags flags{};
  flags.enable.fill(1);

  EXPECT_EQ(monitor.Update(state, flags, 0, true).head_fault_mask[kHeadPitchIndex], 0);
  state.q[kHeadPitchIndex] = 0.4;
  const auto jumped = monitor.Update(state, flags, 0, true, nullptr, 0.002);

  EXPECT_EQ(jumped.head_fault_mask[kHeadPitchIndex], 1);
  EXPECT_NE(jumped.reason_mask & kReasonFeedbackJump, 0U);
  EXPECT_NEAR(jumped.sanitized_state.q[kHeadPitchIndex], 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(jumped.sanitized_state.qd[kHeadPitchIndex], 0.0);
}

TEST(HeadJointHealthMonitorTest, RejectsHeadTrackingStallAfterConfirmationWindow) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  HeadJointHealthMonitor monitor(envelope);
  T800JointState state{};
  state.q.fill(0.0);
  state.qd.fill(0.0);
  T800MotorFlags flags{};
  flags.enable.fill(1);
  T800CommandFrame command{};
  command.q_des[kHeadYawIndex] = 0.3;
  command.kp[kHeadYawIndex] = 10.0;

  for (int frame = 1; frame < kHeadTrackingFaultFrames; ++frame) {
    const auto pending = monitor.Update(state, flags, 0, true, &command, 0.002);
    EXPECT_EQ(pending.head_fault_mask[kHeadYawIndex], 0);
  }
  const auto stalled = monitor.Update(state, flags, 0, true, &command, 0.002);
  EXPECT_EQ(stalled.head_fault_mask[kHeadYawIndex], 1);
  EXPECT_NE(stalled.reason_mask & kReasonFeedbackTracking, 0U);
}

TEST(T800SafetyGateTest, RejectsHardVelocityNegativeGainAndStalledPeriod) {
  T800JointEnvelope envelope; std::string error; ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  JointCommandSafetyGate gate(envelope); T800JointState measured{}; measured.q.fill(0); measured.qd.fill(0);
  T800CommandFrame command{}; command.q_des.fill(0); command.qd_des.fill(0); command.kp.fill(0); command.kd.fill(0); command.tau_ff.fill(0);
  measured.qd[0] = envelope.joint(0).hard_velocity + 1; auto velocity = gate.Evaluate(command, measured, 0.002); EXPECT_FALSE(velocity.accepted); EXPECT_NE(velocity.reason_mask & kReasonVelocity, 0U);
  measured.qd[0] = 0; command.kp[0] = -1; auto gain = gate.Evaluate(command, measured, 0.002); EXPECT_FALSE(gain.accepted); EXPECT_NE(gain.reason_mask & kReasonGain, 0U);
  command.kp[0] = 0; auto late = gate.Evaluate(command, measured, 0.021); EXPECT_FALSE(late.accepted); EXPECT_NE(late.reason_mask & kReasonDeadline, 0U);
}

TEST(T800SafetyGateTest, SlewLimitsTheFirstAcceptedCommandFromMeasuredState) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  JointCommandSafetyGate gate(envelope);
  T800JointState measured{};
  measured.q.fill(0.0);
  measured.qd.fill(0.0);
  T800CommandFrame command{};
  command.q_des.fill(0.0);
  command.qd_des.fill(0.0);
  command.kp.fill(0.0);
  command.kd.fill(0.0);
  command.tau_ff.fill(0.0);
  command.q_des[0] = 1.0;

  const auto result = gate.Evaluate(command, measured, 0.002);
  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.decision, SafetyDecision::kClipped);
  EXPECT_NE(result.reason_mask & kReasonSlew, 0U);
  EXPECT_NEAR(result.frame.q_des[0], envelope.joint(0).operational_velocity * 0.002,
              1e-12);
}

TEST(T800MotorCommandSafetyGateTest, RejectsUnsafeTransformedEffort) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  MotorCommandSafetyGate gate(envelope);
  T800JointState measured{};
  T800CommandFrame command{};

  EXPECT_TRUE(gate.Evaluate(command, measured).accepted);
  command.tau_ff[0] = envelope.joint(0).operational_effort + 1.0;
  const auto excessive_feed_forward = gate.Evaluate(command, measured);
  EXPECT_FALSE(excessive_feed_forward.accepted);
  EXPECT_NE(excessive_feed_forward.reason_mask & kReasonEffort, 0U);

  command.tau_ff[0] = 0.0;
  command.q_des[0] = 0.2;
  command.kp[0] = envelope.joint(0).operational_effort / 0.1;
  const auto excessive_pd = gate.Evaluate(command, measured);
  EXPECT_FALSE(excessive_pd.accepted);
  EXPECT_NE(excessive_pd.reason_mask & kReasonPdEstimate, 0U);
}

TEST(T800MotorCommandSafetyGateTest, RejectsUnsafeDirectDrivePositionAndVelocity) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  MotorCommandSafetyGate gate(envelope);
  T800JointState measured{};
  T800CommandFrame command{};

  command.q_des[0] = envelope.joint(0).operational_position_upper + 0.01;
  const auto excessive_position = gate.Evaluate(command, measured);
  EXPECT_FALSE(excessive_position.accepted);
  EXPECT_NE(excessive_position.reason_mask & kReasonHardPosition, 0U);

  command.q_des[0] = 0.0;
  command.qd_des[0] = envelope.joint(0).operational_velocity + 0.01;
  const auto excessive_velocity = gate.Evaluate(command, measured);
  EXPECT_FALSE(excessive_velocity.accepted);
  EXPECT_NE(excessive_velocity.reason_mask & kReasonVelocity, 0U);

  command.qd_des[0] = 0.0;
  measured.q[0] = envelope.joint(0).hard_position_upper + 0.01;
  const auto invalid_measured_position = gate.Evaluate(command, measured);
  EXPECT_FALSE(invalid_measured_position.accepted);
  EXPECT_NE(invalid_measured_position.reason_mask & kReasonHardPosition, 0U);

  measured.q[0] = 0.0;
  measured.qd[0] = envelope.joint(0).hard_velocity + 0.01;
  const auto invalid_measured_velocity = gate.Evaluate(command, measured);
  EXPECT_FALSE(invalid_measured_velocity.accepted);
  EXPECT_NE(invalid_measured_velocity.reason_mask & kReasonVelocity, 0U);
}

TEST(T800MotorCommandSafetyGateTest, DoesNotTreatJointEffortAsParallelMotorLimit) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  MotorCommandSafetyGate gate(envelope);
  T800JointState measured{};
  T800CommandFrame command{};
  command.tau_ff[4] = envelope.joint(4).operational_effort + 1.0;
  command.q_des[4] = envelope.joint(4).operational_position_upper + 1.0;
  command.qd_des[4] = envelope.joint(4).operational_velocity + 1.0;
  measured.q[4] = envelope.joint(4).hard_position_upper + 1.0;
  measured.qd[4] = envelope.joint(4).hard_velocity + 1.0;

  // J04/J05/J10/J11 are nonlinear parallel-ankle motor channels after the
  // transform. Rejecting them against a joint-space URDF effort would create a
  // false safety claim (and can create unsafe nuisance trips).
  EXPECT_TRUE(gate.Evaluate(command, measured).accepted);
}

TEST(T800SanitizedStateTest, FailsClosedBeforeFirstResidentSnapshot) {
  auto& store = data::VariantStore::GetInstance();
  store.Remove("safety/t800_snapshot");
  Eigen::VectorXd q = Eigen::VectorXd::LinSpaced(kJointCount, -0.2, 0.2);
  Eigen::VectorXd qd = Eigen::VectorXd::Constant(kJointCount, 0.1);
  const Eigen::VectorXd expected_q = q;
  const Eigen::VectorXd expected_qd = qd;
  T800SafetySnapshot snapshot;

  EXPECT_FALSE(GetT800SanitizedState(&q, &qd, &snapshot));
  EXPECT_TRUE(q.isApprox(expected_q));
  EXPECT_TRUE(qd.isApprox(expected_qd));
  EXPECT_TRUE(snapshot.frame_fault);
  EXPECT_NE(snapshot.reason_mask & kReasonDeadline, 0U);
  EXPECT_NE(snapshot.reason_mask & kReasonFrameFault, 0U);
}

TEST(T800SanitizedStateTest, RejectsStaleSnapshotWithoutCopyingItsState) {
  auto& store = data::VariantStore::GetInstance();
  T800SafetySnapshot stale;
  stale.sanitized_state.q.fill(0.75);
  stale.sanitized_state.qd.fill(0.5);
  stale.published_at_steady_ns = 1;
  store.Set("safety/t800_snapshot", stale);
  Eigen::VectorXd q = Eigen::VectorXd::Constant(kJointCount, 0.1);
  Eigen::VectorXd qd = Eigen::VectorXd::Zero(kJointCount);
  T800SafetySnapshot result;

  EXPECT_FALSE(GetT800SanitizedState(&q, &qd, &result));
  EXPECT_TRUE(result.frame_fault);
  EXPECT_NE(result.reason_mask & kReasonDeadline, 0U);
  EXPECT_TRUE(q.isApprox(Eigen::VectorXd::Constant(kJointCount, 0.1)));
  store.Remove("safety/t800_snapshot");
}

TEST(T800HeadActionMaskTest, ClearsOnlyActionsMappedToFailedHeadJoints) {
  T800SafetySnapshot snapshot;
  snapshot.head_fault_mask[kHeadPitchIndex] = 1;
  snapshot.head_fault_mask[kHeadYawIndex] = 1;
  Eigen::VectorXi action_to_joint(4);
  action_to_joint << static_cast<int>(kHeadYawIndex), 3,
      static_cast<int>(kHeadPitchIndex), 7;
  Eigen::VectorXd actions(4);
  actions << 1.0, 2.0, 3.0, 4.0;

  MaskFailedHeadActions(snapshot, action_to_joint, &actions);

  EXPECT_DOUBLE_EQ(actions[0], 0.0);
  EXPECT_DOUBLE_EQ(actions[1], 2.0);
  EXPECT_DOUBLE_EQ(actions[2], 0.0);
  EXPECT_DOUBLE_EQ(actions[3], 4.0);
}

}  // namespace
}  // namespace runner::t800_safety

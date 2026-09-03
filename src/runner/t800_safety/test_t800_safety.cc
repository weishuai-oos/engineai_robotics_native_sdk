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
  EXPECT_DOUBLE_EQ(envelope.recovery_limits().position_overrun_rad, 0.01);
  EXPECT_DOUBLE_EQ(envelope.recovery_limits().max_outward_velocity_rad_s, 0.2);
  EXPECT_EQ(envelope.recovery_limits().healthy_frames, 50);
}

TEST(T800SafetyStatusTest, ExposesReadableStatusAndMotionProfiles) {
  EXPECT_EQ(SafetyStatusMessage(SafetyStatus::kNormal),
            "NORMAL：状态正常，命令在安全范围内");
  EXPECT_EQ(SafetyStatusMessage(SafetyStatus::kLimited),
            "LIMITED：轻微越界或瞬时命令裁剪，仍允许受控运动");
  EXPECT_EQ(SafetyStatusMessage(SafetyStatus::kRecovery),
            "RECOVERY：检测到可恢复越界，仅允许向安全区方向运动");
  EXPECT_EQ(SafetyStatusMessage(SafetyStatus::kFatal),
            "FATAL：检测到不可恢复或数据/电机故障，已停止驱动输出");
  EXPECT_EQ(SafetyProfileForMotion("walk_leo"), SafetyProfile::kPolicy);
  EXPECT_EQ(SafetyProfileForMotion("getup"), SafetyProfile::kGetup);
  EXPECT_EQ(SafetyProfileForMotion("pd_stand"),
            SafetyProfile::kPdStandRecovery);
  EXPECT_EQ(MotorEnablePhaseMessage(MotorEnablePhase::kEnabling),
            "电机使能中，安全输出保持为零");
}

TEST(T800MotorEnableGuardTest, AllowsArmingButFailsClosedAfterDropout) {
  MotorEnableGuard guard;
  constexpr std::uint64_t start = 1'000'000'000;

  EXPECT_EQ(guard.Update(false, false, false, start),
            MotorEnablePhase::kIntentionallyDisabled);
  EXPECT_EQ(guard.Update(true, false, false, start + 1),
            MotorEnablePhase::kEnabling);
  EXPECT_EQ(guard.Update(true, true, true, start + 2),
            MotorEnablePhase::kReady);
  EXPECT_EQ(guard.Update(true, true, false, start + 3),
            MotorEnablePhase::kFault);
}

TEST(T800MotorEnableGuardTest, ArmingTimeoutDoesNotBecomePermanentLimited) {
  MotorEnableGuard guard;
  constexpr std::uint64_t start = 1'000'000'000;

  EXPECT_EQ(guard.Update(true, false, false, start),
            MotorEnablePhase::kEnabling);
  EXPECT_EQ(guard.Update(true, false, false,
                         start + kMotorEnableGraceNs),
            MotorEnablePhase::kEnabling);
  EXPECT_EQ(guard.Update(true, false, false,
                         start + kMotorEnableGraceNs + 1),
            MotorEnablePhase::kFault);
}

TEST(T800SafetyStatusTest, PrivilegedProfileRequiresMatchingFreshLease) {
  ReleaseSafetyProfile(SafetyProfile::kGetup);
  ReleaseSafetyProfile(SafetyProfile::kPdStandRecovery);
  bool downgraded = false;

  EXPECT_EQ(ResolveSafetyProfileForMotion("getup", &downgraded),
            SafetyProfile::kPolicy);
  EXPECT_TRUE(downgraded);

  RequestSafetyProfile(SafetyProfile::kGetup);
  EXPECT_EQ(ResolveSafetyProfileForMotion("getup", &downgraded),
            SafetyProfile::kGetup);
  EXPECT_FALSE(downgraded);
  EXPECT_EQ(ResolveSafetyProfileForMotion("pd_stand", &downgraded),
            SafetyProfile::kPolicy);
  EXPECT_TRUE(downgraded);

  ReleaseSafetyProfile(SafetyProfile::kGetup);
  SafetyProfileLease stale;
  stale.profile = SafetyProfile::kGetup;
  stale.published_at_steady_ns = 1;
  data::VariantStore::GetInstance().Set("safety/t800_profile_lease", stale);
  EXPECT_EQ(ResolveSafetyProfileForMotion("getup", &downgraded),
            SafetyProfile::kPolicy);
  EXPECT_TRUE(downgraded);

  RequestSafetyProfile(SafetyProfile::kPdStandRecovery);
  EXPECT_EQ(ResolveSafetyProfileForMotion("pd_stand", &downgraded),
            SafetyProfile::kPdStandRecovery);
  EXPECT_FALSE(downgraded);
  ReleaseSafetyProfile(SafetyProfile::kPdStandRecovery);
}

TEST(T800SafetyGateTest, ClipsLimitsAndSlewAndRejectsFullTorque) {
  T800JointEnvelope envelope; std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  JointCommandSafetyGate gate(envelope); T800JointState measured; measured.q.fill(0); measured.qd.fill(0);
  T800CommandFrame command; command.q_des.fill(0); command.qd_des.fill(0); command.kp.fill(0); command.kd.fill(0); command.tau_ff.fill(0);
  command.q_des[0] = envelope.joint(0).operational_position_upper + 1; command.qd_des[0] = envelope.joint(0).operational_velocity + 1;
  auto first = gate.Evaluate(command, measured, 0.02); ASSERT_TRUE(first.accepted); EXPECT_EQ(first.decision, SafetyDecision::kClipped);
  EXPECT_EQ(first.status, SafetyStatus::kLimited);
  EXPECT_NE(first.reason_mask & kReasonOperationalPosition, 0U);
  command.full_torque_enabled = true; auto rejected = gate.Evaluate(command, measured, 0.02); EXPECT_FALSE(rejected.accepted); EXPECT_NE(rejected.reason_mask & kReasonFullTorqueUnsupported, 0U);
}

TEST(T800SafetyGateTest, RecoveryProfilesUseHardPositionButOperationalDynamics) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  const auto& joint = envelope.joint(0);
  T800JointState measured{};
  measured.q[0] = joint.operational_position_upper;
  T800CommandFrame command{};
  command.q_des[0] =
      0.5 * (joint.operational_position_upper + joint.hard_position_upper);

  JointCommandSafetyGate policy_gate(envelope);
  const auto policy =
      policy_gate.Evaluate(command, measured, 0.02, SafetyProfile::kPolicy);
  ASSERT_TRUE(policy.accepted);
  EXPECT_EQ(policy.status, SafetyStatus::kLimited);
  EXPECT_DOUBLE_EQ(policy.frame.q_des[0], joint.operational_position_upper);

  JointCommandSafetyGate getup_gate(envelope);
  const auto getup =
      getup_gate.Evaluate(command, measured, 0.02, SafetyProfile::kGetup);
  ASSERT_TRUE(getup.accepted);
  EXPECT_DOUBLE_EQ(getup.frame.q_des[0], command.q_des[0]);
  EXPECT_LE(std::abs(getup.frame.qd_des[0]), joint.operational_velocity);

  MotorCommandSafetyGate motor_gate(envelope);
  EXPECT_TRUE(
      motor_gate.Evaluate(getup.frame, measured, SafetyProfile::kGetup).accepted);

  command.qd_des[0] = joint.operational_velocity + 1.0;
  const auto dynamic_limited =
      getup_gate.Evaluate(command, measured, 0.02, SafetyProfile::kGetup);
  ASSERT_TRUE(dynamic_limited.accepted);
  EXPECT_EQ(dynamic_limited.status, SafetyStatus::kLimited);
  EXPECT_DOUBLE_EQ(dynamic_limited.frame.qd_des[0], joint.operational_velocity);
  EXPECT_FALSE(
      motor_gate.Evaluate(command, measured, SafetyProfile::kGetup).accepted);
}

TEST(T800SafetyGateTest, LimitsEstimatedPdEffortWithoutFreezingMotion) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  JointCommandSafetyGate gate(envelope);
  T800JointState measured{};
  T800CommandFrame command{};
  command.q_des[0] = 0.2;
  command.kp[0] = envelope.joint(0).operational_effort / 0.1;

  const auto result = gate.Evaluate(command, measured, 0.02);

  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.status, SafetyStatus::kLimited);
  EXPECT_NE(result.reason_mask & kReasonPdEstimate, 0U);
  const double estimated_effort =
      result.frame.kp[0] * (result.frame.q_des[0] - measured.q[0]);
  EXPECT_LE(std::abs(estimated_effort),
            envelope.joint(0).operational_effort + 1e-9);
}

TEST(T800SafetyGateTest, LimitsDerivativeEffortByClippingVelocityNotAddingTorque) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  JointCommandSafetyGate gate(envelope);
  T800JointState measured{};
  T800CommandFrame command{};
  const double effort = envelope.joint(0).operational_effort;
  command.qd_des[0] = 0.2;
  command.kd[0] = effort / 0.1;

  const auto result = gate.Evaluate(command, measured, 0.02);

  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.status, SafetyStatus::kLimited);
  EXPECT_NE(result.reason_mask & kReasonPdEstimate, 0U);
  EXPECT_NEAR(result.frame.qd_des[0], 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(result.frame.tau_ff[0], 0.0);
  const double estimated_effort =
      result.frame.kp[0] * (result.frame.q_des[0] - measured.q[0]) +
      result.frame.kd[0] * (result.frame.qd_des[0] - measured.qd[0]) +
      result.frame.tau_ff[0];
  EXPECT_LE(std::abs(estimated_effort), effort + 1e-9);
}

TEST(HeadJointHealthMonitorTest, SanitizesHeadWithoutHidingFrameFault) {
  T800JointEnvelope envelope; std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  HeadJointHealthMonitor monitor(envelope); T800JointState state; state.q.fill(0); state.qd.fill(0); state.tau.fill(0);
  T800MotorFlags flags; flags.enable.fill(1); auto good = monitor.Update(state, flags); EXPECT_FALSE(good.frame_fault);
  state.q[kHeadPitchIndex] = std::numeric_limits<double>::quiet_NaN(); state.q[0] = std::numeric_limits<double>::quiet_NaN();
  auto bad = monitor.Update(state, flags); EXPECT_EQ(bad.head_fault_mask[kHeadPitchIndex], 1); EXPECT_TRUE(bad.frame_fault); EXPECT_TRUE(std::isfinite(bad.sanitized_state.q[kHeadPitchIndex]));
}

TEST(HeadJointHealthMonitorTest, IntentionalDisableDoesNotPoisonRecoveryEntry) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error))
      << error;
  T800JointState state{};
  T800MotorFlags disabled{};

  HeadJointHealthMonitor strict_monitor(envelope);
  const auto strict = strict_monitor.Update(state, disabled, 0, true);
  EXPECT_TRUE(strict.frame_fault);
  EXPECT_EQ(strict.status, SafetyStatus::kFatal);

  HeadJointHealthMonitor transition_monitor(envelope);
  const auto transition = transition_monitor.Update(
      state, disabled, 0, true, nullptr, 0.002,
      /*disabled_motor_is_fault=*/false);
  EXPECT_FALSE(transition.frame_fault);
  EXPECT_EQ(transition.status, SafetyStatus::kNormal);
  EXPECT_EQ(transition.reason_mask, kReasonNone);
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

TEST(HeadJointHealthMonitorTest, AllowsSmallHardLimitOverrunToRecoverInward) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  HeadJointHealthMonitor monitor(envelope);
  T800JointState state{};
  T800MotorFlags flags{};
  flags.enable.fill(1);
  ASSERT_EQ(monitor.Update(state, flags, 0, true).status,
            SafetyStatus::kNormal);

  const double overrun = 0.5 * envelope.recovery_limits().position_overrun_rad;
  state.q[0] = envelope.joint(0).hard_position_upper + overrun;
  const auto recovering = monitor.Update(state, flags, 0, true);

  EXPECT_FALSE(recovering.frame_fault);
  EXPECT_EQ(recovering.status, SafetyStatus::kRecovery);
  EXPECT_EQ(recovering.health[0], JointHealth::kRecovering);
  EXPECT_EQ(recovering.recovery_direction[0], RecoveryDirection::kDecrease);
  EXPECT_DOUBLE_EQ(recovering.sanitized_state.q[0], state.q[0]);
  EXPECT_NE(recovering.reason_mask & kReasonRecoverablePosition, 0U);

  state.q[0] = envelope.joint(0).hard_position_upper;
  for (int frame = 1; frame < envelope.recovery_limits().healthy_frames;
       ++frame) {
    const auto hysteresis = monitor.Update(state, flags, 0, true);
    EXPECT_EQ(hysteresis.status, SafetyStatus::kRecovery);
    EXPECT_EQ(hysteresis.recovery_direction[0], RecoveryDirection::kDecrease);
  }
  const auto healthy = monitor.Update(state, flags, 0, true);
  EXPECT_EQ(healthy.status, SafetyStatus::kNormal);
  EXPECT_EQ(healthy.health[0], JointHealth::kHealthy);
  EXPECT_EQ(healthy.recovery_direction[0], RecoveryDirection::kNone);
}

TEST(T800SafetyGateTest, RecoveryHysteresisKeepsBlockingOutwardCommands) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  const auto& joint = envelope.joint(0);
  HeadJointHealthMonitor monitor(envelope);
  T800MotorFlags flags{};
  flags.enable.fill(1);
  T800JointState state{};
  state.q[0] = joint.hard_position_upper +
               0.5 * envelope.recovery_limits().position_overrun_rad;
  ASSERT_EQ(monitor.Update(state, flags, 0, true).status,
            SafetyStatus::kRecovery);

  state.q[0] = joint.hard_position_upper - 1e-3;
  const auto hysteresis = monitor.Update(state, flags, 0, true);
  ASSERT_EQ(hysteresis.status, SafetyStatus::kRecovery);
  ASSERT_EQ(hysteresis.recovery_direction[0], RecoveryDirection::kDecrease);

  T800CommandFrame outward{};
  outward.q_des[0] = joint.hard_position_upper;
  outward.qd_des[0] = 0.1;
  outward.tau_ff[0] = 1.0;
  JointCommandSafetyGate joint_gate(envelope);
  const auto clipped = joint_gate.Evaluate(
      outward, state, 0.002, SafetyProfile::kGetup,
      &hysteresis.recovery_direction);
  ASSERT_TRUE(clipped.accepted);
  EXPECT_EQ(clipped.status, SafetyStatus::kRecovery);
  EXPECT_LE(clipped.frame.q_des[0], state.q[0]);
  EXPECT_LE(clipped.frame.qd_des[0], 0.0);
  EXPECT_LE(clipped.frame.tau_ff[0], 0.0);

  MotorCommandSafetyGate motor_gate(envelope);
  const auto rejected = motor_gate.Evaluate(
      outward, state, SafetyProfile::kGetup,
      &hysteresis.recovery_direction);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_NE(rejected.reason_mask & kReasonHardPosition, 0U);
  const auto accepted = motor_gate.Evaluate(
      clipped.frame, state, SafetyProfile::kGetup,
      &hysteresis.recovery_direction);
  EXPECT_TRUE(accepted.accepted);
  EXPECT_EQ(accepted.status, SafetyStatus::kRecovery);
}

TEST(HeadJointHealthMonitorTest, RejectsSevereOrFastOutwardHardLimitOverrun) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  T800MotorFlags flags{};
  flags.enable.fill(1);

  HeadJointHealthMonitor boundary_monitor(envelope);
  T800JointState boundary{};
  boundary.q[0] = envelope.joint(0).hard_position_upper +
                  envelope.recovery_limits().position_overrun_rad;
  boundary.qd[0] =
      envelope.recovery_limits().max_outward_velocity_rad_s;
  const auto boundary_result = boundary_monitor.Update(boundary, flags, 0, true);
  EXPECT_FALSE(boundary_result.frame_fault);
  EXPECT_EQ(boundary_result.status, SafetyStatus::kRecovery);

  HeadJointHealthMonitor severe_monitor(envelope);
  T800JointState severe{};
  severe.q[0] = envelope.joint(0).hard_position_upper +
                envelope.recovery_limits().position_overrun_rad + 1e-4;
  const auto severe_result = severe_monitor.Update(severe, flags, 0, true);
  EXPECT_TRUE(severe_result.frame_fault);
  EXPECT_EQ(severe_result.status, SafetyStatus::kFatal);
  EXPECT_EQ(severe_result.affected_joint_mask[0], 1);
  EXPECT_NE(severe_result.reason_mask & kReasonHardPosition, 0U);
  EXPECT_DOUBLE_EQ(severe_result.sanitized_state.qd[0], 0.0);

  HeadJointHealthMonitor severe_lower_monitor(envelope);
  T800JointState severe_lower{};
  severe_lower.q[0] = envelope.joint(0).hard_position_lower -
                      envelope.recovery_limits().position_overrun_rad - 1e-4;
  const auto severe_lower_result =
      severe_lower_monitor.Update(severe_lower, flags, 0, true);
  EXPECT_TRUE(severe_lower_result.frame_fault);
  EXPECT_EQ(severe_lower_result.status, SafetyStatus::kFatal);

  HeadJointHealthMonitor outward_monitor(envelope);
  T800JointState outward{};
  outward.q[0] = envelope.joint(0).hard_position_upper +
                 0.5 * envelope.recovery_limits().position_overrun_rad;
  outward.qd[0] =
      envelope.recovery_limits().max_outward_velocity_rad_s + 0.01;
  const auto outward_result = outward_monitor.Update(outward, flags, 0, true);
  EXPECT_TRUE(outward_result.frame_fault);
  EXPECT_EQ(outward_result.status, SafetyStatus::kFatal);
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

TEST(T800SafetyGateTest, SmallHardLimitOverrunOnlyCommandsTowardSafeInterval) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  const auto& joint = envelope.joint(0);
  T800JointState measured{};
  measured.q[0] = joint.hard_position_upper +
                  0.5 * envelope.recovery_limits().position_overrun_rad;
  T800CommandFrame command{};
  command.q_des[0] = measured.q[0] + 0.1;
  command.qd_des[0] = 1.0;
  command.tau_ff[0] = 10.0;

  JointCommandSafetyGate gate(envelope);
  const auto result =
      gate.Evaluate(command, measured, 0.002, SafetyProfile::kGetup);

  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.status, SafetyStatus::kRecovery);
  EXPECT_EQ(result.decision, SafetyDecision::kRecovery);
  EXPECT_EQ(result.recovery_direction[0], RecoveryDirection::kDecrease);
  EXPECT_LT(result.frame.q_des[0], measured.q[0]);
  EXPECT_LE(result.frame.qd_des[0], 0.0);
  EXPECT_LE(result.frame.tau_ff[0], 0.0);

  MotorCommandSafetyGate motor_gate(envelope);
  const auto inward =
      motor_gate.Evaluate(result.frame, measured, SafetyProfile::kGetup);
  EXPECT_TRUE(inward.accepted);
  EXPECT_EQ(inward.status, SafetyStatus::kRecovery);

  T800CommandFrame outward = result.frame;
  outward.q_des[0] = measured.q[0] + 1e-3;
  const auto rejected =
      motor_gate.Evaluate(outward, measured, SafetyProfile::kGetup);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_NE(rejected.reason_mask & kReasonHardPosition, 0U);

  outward = result.frame;
  outward.tau_ff[0] = 1.0;
  const auto rejected_outward_torque =
      motor_gate.Evaluate(outward, measured, SafetyProfile::kGetup);
  EXPECT_FALSE(rejected_outward_torque.accepted);
  EXPECT_NE(rejected_outward_torque.reason_mask & kReasonEffort, 0U);
}

TEST(T800SafetyGateTest, LowerHardLimitRecoveryIsSymmetric) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  const auto& joint = envelope.joint(0);
  T800JointState measured{};
  measured.q[0] = joint.hard_position_lower -
                  0.5 * envelope.recovery_limits().position_overrun_rad;
  T800CommandFrame command{};
  command.q_des[0] = measured.q[0] - 0.1;
  command.qd_des[0] = -1.0;
  JointCommandSafetyGate gate(envelope);

  const auto result =
      gate.Evaluate(command, measured, 0.002, SafetyProfile::kPdStandRecovery);

  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.status, SafetyStatus::kRecovery);
  EXPECT_EQ(result.recovery_direction[0], RecoveryDirection::kIncrease);
  EXPECT_GT(result.frame.q_des[0], measured.q[0]);
  EXPECT_GE(result.frame.qd_des[0], 0.0);
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

  command = {};
  measured.tau[0] = std::numeric_limits<double>::quiet_NaN();
  const auto non_finite_feedback = gate.Evaluate(command, measured);
  EXPECT_FALSE(non_finite_feedback.accepted);
  EXPECT_EQ(non_finite_feedback.status, SafetyStatus::kFatal);
  EXPECT_NE(non_finite_feedback.reason_mask & kReasonNonFinite, 0U);
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
  EXPECT_NE(excessive_position.reason_mask & kReasonOperationalPosition, 0U);

  command.q_des[0] = 0.0;
  command.qd_des[0] = envelope.joint(0).operational_velocity + 0.01;
  const auto excessive_velocity = gate.Evaluate(command, measured);
  EXPECT_FALSE(excessive_velocity.accepted);
  EXPECT_NE(excessive_velocity.reason_mask & kReasonVelocity, 0U);

  command.qd_des[0] = 0.0;
  measured.q[0] = envelope.joint(0).hard_position_upper +
                  envelope.recovery_limits().position_overrun_rad + 1e-4;
  const auto invalid_measured_position = gate.Evaluate(command, measured);
  EXPECT_FALSE(invalid_measured_position.accepted);
  EXPECT_NE(invalid_measured_position.reason_mask & kReasonHardPosition, 0U);

  measured.q[0] = 0.0;
  measured.qd[0] = envelope.joint(0).hard_velocity + 0.01;
  const auto invalid_measured_velocity = gate.Evaluate(command, measured);
  EXPECT_FALSE(invalid_measured_velocity.accepted);
  EXPECT_NE(invalid_measured_velocity.reason_mask & kReasonVelocity, 0U);
}

TEST(T800MotorCommandSafetyGateTest, PolicyEnvelopeRecoveryReportsLimited) {
  T800JointEnvelope envelope;
  std::string error;
  ASSERT_TRUE(T800JointEnvelope::Load(ContractPath(), &envelope, &error)) << error;
  const auto& joint = envelope.joint(0);
  MotorCommandSafetyGate gate(envelope);
  T800JointState measured{};
  measured.q[0] =
      0.5 * (joint.operational_position_upper + joint.hard_position_upper);
  T800CommandFrame command{};
  command.q_des[0] = joint.operational_position_upper;

  const auto result = gate.Evaluate(command, measured, SafetyProfile::kPolicy);

  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.status, SafetyStatus::kLimited);
  EXPECT_EQ(result.recovery_direction[0], RecoveryDirection::kDecrease);
  EXPECT_NE(result.reason_mask & kReasonOperationalPosition, 0U);
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

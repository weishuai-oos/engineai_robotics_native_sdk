#include "t800_safety/t800_safety_runner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "data/variant_store/variant_store.h"
#include "motor_debug/motor_debug.h"
#include "parameter/global_config_initializer.h"
#include "tool/string_join.h"

namespace runner::t800_safety {
namespace {
constexpr double kEpsilon = 1e-9;
constexpr double kOperationalScale = 0.9;
constexpr std::uint64_t kSnapshotFreshnessLimitNs = 50'000'000;
constexpr std::uint64_t kMotorDebugFreshnessLimitNs = 100'000'000;
constexpr double kHeadJumpVelocityMultiplier = 1.25;
constexpr double kHeadJumpSlackRad = 0.01;
constexpr double kHeadStationaryDeltaRad = 1e-5;
constexpr double kHeadVelocityMismatchRadS = 0.25;
constexpr double kHeadTrackingErrorRad = 0.15;
constexpr double kHeadTrackingVelocityRadS = 0.02;
constexpr double kHeadTrackingMinimumKp = 1.0;
constexpr std::string_view kSafetyProfileLeaseKey =
    "safety/t800_profile_lease";
std::uint64_t SteadyNowNs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}
void Error(std::string* error, const std::string& message) { if (error) *error = message; }
bool Scalar(const YAML::Node& n, const char* key, double* out, std::string* error) {
  if (!n[key] || !n[key].IsScalar()) { Error(error, std::string("missing ") + key); return false; }
  try { *out = n[key].as<double>(); } catch (const YAML::Exception& e) { Error(error, e.what()); return false; }
  if (!std::isfinite(*out)) { Error(error, std::string("non-finite ") + key); return false; }
  return true;
}
bool Text(const YAML::Node& n, const char* key, std::string* out, std::string* error) {
  if (!n[key] || !n[key].IsScalar()) { Error(error, std::string("missing ") + key); return false; }
  try { *out = n[key].as<std::string>(); } catch (const YAML::Exception& e) { Error(error, e.what()); return false; }
  if (out->empty()) { Error(error, std::string("empty ") + key); return false; }
  return true;
}
bool PositiveInteger(const YAML::Node& n, const char* key, int* out,
                     std::string* error) {
  if (!n[key] || !n[key].IsScalar()) {
    Error(error, std::string("missing ") + key);
    return false;
  }
  try {
    *out = n[key].as<int>();
  } catch (const YAML::Exception& e) {
    Error(error, e.what());
    return false;
  }
  if (*out <= 0) {
    Error(error, std::string("non-positive ") + key);
    return false;
  }
  return true;
}
bool Finite(const std::array<double, kJointCount>& a) {
  for (double v : a) if (!std::isfinite(v)) return false;
  return true;
}
bool Head(std::size_t i) { return i == kHeadPitchIndex || i == kHeadYawIndex; }
bool ParallelAnkle(std::size_t i) { return i == 4 || i == 5 || i == 10 || i == 11; }

double PositionOverrun(const JointEnvelope& joint, double position,
                       RecoveryDirection* direction) {
  if (direction) *direction = RecoveryDirection::kNone;
  if (position < joint.hard_position_lower) {
    if (direction) *direction = RecoveryDirection::kIncrease;
    return joint.hard_position_lower - position;
  }
  if (position > joint.hard_position_upper) {
    if (direction) *direction = RecoveryDirection::kDecrease;
    return position - joint.hard_position_upper;
  }
  return 0.0;
}

double OutwardVelocity(RecoveryDirection direction, double velocity) {
  if (direction == RecoveryDirection::kIncrease) return std::max(0.0, -velocity);
  if (direction == RecoveryDirection::kDecrease) return std::max(0.0, velocity);
  return 0.0;
}

bool RecoverablePositionOverrun(const T800JointEnvelope& envelope,
                                const JointEnvelope& joint, double position,
                                double velocity, RecoveryDirection* direction) {
  const double overrun = PositionOverrun(joint, position, direction);
  return overrun > 0.0 &&
         overrun <= envelope.recovery_limits().position_overrun_rad + kEpsilon &&
         OutwardVelocity(*direction, velocity) <=
             envelope.recovery_limits().max_outward_velocity_rad_s + kEpsilon;
}

bool UsesHardPositionEnvelope(SafetyProfile profile) {
  return profile == SafetyProfile::kGetup ||
         profile == SafetyProfile::kPdStandRecovery;
}

double PositionLower(const JointEnvelope& joint, SafetyProfile profile) {
  return UsesHardPositionEnvelope(profile) ? joint.hard_position_lower
                                           : joint.operational_position_lower;
}

double PositionUpper(const JointEnvelope& joint, SafetyProfile profile) {
  return UsesHardPositionEnvelope(profile) ? joint.hard_position_upper
                                           : joint.operational_position_upper;
}

T800SafetySnapshot SanitizeRawState(const Eigen::VectorXd& q, const Eigen::VectorXd& qd) {
  T800SafetySnapshot local;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const Eigen::Index index = static_cast<Eigen::Index>(i);
    const bool finite = std::isfinite(q[index]) && std::isfinite(qd[index]);
    if (finite) {
      local.sanitized_state.q[i] = q[index];
      local.sanitized_state.qd[i] = qd[index];
      local.health[i] = JointHealth::kHealthy;
      continue;
    }
    local.sanitized_state.q[i] = 0.0;
    local.sanitized_state.qd[i] = 0.0;
    local.health[i] = Head(i) ? JointHealth::kFailed : JointHealth::kSuspect;
    local.reason_mask |= kReasonNonFinite;
    if (Head(i)) {
      local.head_fault_mask[i] = 1;
    } else {
      local.status = SafetyStatus::kFatal;
      local.frame_fault = true;
      local.reason_mask |= kReasonFrameFault;
    }
  }
  return local;
}
}

bool T800JointEnvelope::Load(const std::string& path, T800JointEnvelope* out, std::string* error) {
  if (!out) { Error(error, "null output"); return false; }
  try {
    const YAML::Node root = YAML::LoadFile(path);
    T800JointEnvelope value;
    if (!root["schema_version"] || !root["contract_version"] || !root["robot"] ||
        !root["qualification"] || !root["provenance"] || !root["recovery"] ||
        !root["joints"]) {
      Error(error, "missing schema v2 top-level field"); return false;
    }
    value.schema_version_ = root["schema_version"].as<int>();
    if (!Text(root, "contract_version", &value.contract_version_, error) ||
        !Text(root, "robot", &value.robot_, error) ||
        !Text(root, "qualification", &value.qualification_, error) ||
        !root["provenance"].IsMap() ||
        !Text(root["provenance"], "source_urdf_sha256", &value.source_urdf_sha256_, error) ||
        !root["recovery"].IsMap() ||
        !Scalar(root["recovery"], "position_overrun_rad",
                &value.recovery_limits_.position_overrun_rad, error) ||
        !Scalar(root["recovery"], "max_outward_velocity_rad_s",
                &value.recovery_limits_.max_outward_velocity_rad_s, error) ||
        !PositiveInteger(root["recovery"], "healthy_frames",
                         &value.recovery_limits_.healthy_frames, error) ||
        !root["joints"].IsSequence() || root["joints"].size() != kJointCount) return false;
    for (const auto& node : root["joints"]) {
      if (!node["sdk_name"] || !node["sdk_index"] || !node["hard_limits"] || !node["operational_limits"]) {
        Error(error, "joint missing v2 fields"); return false;
      }
      int index = node["sdk_index"].as<int>();
      if (index < 0 || index >= static_cast<int>(kJointCount)) { Error(error, "sdk index out of range"); return false; }
      auto& j = value.joints_[static_cast<std::size_t>(index)];
      if (!Text(node, "sdk_name", &j.sdk_name, error)) return false;
      j.sdk_index = index;
      const auto hard = node["hard_limits"]; const auto op = node["operational_limits"];
      if (!Scalar(hard, "position_lower_rad", &j.hard_position_lower, error) ||
          !Scalar(hard, "position_upper_rad", &j.hard_position_upper, error) ||
          !Scalar(hard, "velocity_rad_s", &j.hard_velocity, error) ||
          !Scalar(hard, "effort_nm", &j.hard_effort, error) ||
          !Scalar(op, "position_lower_rad", &j.operational_position_lower, error) ||
          !Scalar(op, "position_upper_rad", &j.operational_position_upper, error) ||
          !Scalar(op, "velocity_rad_s", &j.operational_velocity, error) ||
          !Scalar(op, "effort_nm", &j.operational_effort, error)) return false;
    }
    if (!value.Validate(error)) return false;
    *out = std::move(value); return true;
  } catch (const YAML::Exception& e) { Error(error, std::string("parse error: ") + e.what()); return false; }
}

bool T800JointEnvelope::Validate(std::string* error) const {
  if (schema_version_ != 2 || robot_ != "t800" || qualification_ != "urdf_derived_provisional" ||
      source_urdf_sha256_.size() != 64 ||
      source_urdf_sha256_.find_first_not_of("0123456789abcdef") != std::string::npos ||
      !std::isfinite(recovery_limits_.position_overrun_rad) ||
      recovery_limits_.position_overrun_rad <= 0.0 ||
      !std::isfinite(recovery_limits_.max_outward_velocity_rad_s) ||
      recovery_limits_.max_outward_velocity_rad_s < 0.0 ||
      recovery_limits_.healthy_frames <= 0) {
    Error(error, "invalid T800 v2 header"); return false;
  }
  std::array<bool, kJointCount> seen{};
  for (const auto& j : joints_) {
    if (j.sdk_index < 0 || j.sdk_index >= static_cast<int>(kJointCount) || seen[j.sdk_index] ||
        j.sdk_name.empty() ||
        !(j.hard_position_lower < j.hard_position_upper) || !(j.operational_position_lower < j.operational_position_upper) ||
        j.operational_position_lower < j.hard_position_lower || j.operational_position_upper > j.hard_position_upper ||
        j.operational_position_lower < kOperationalScale * j.hard_position_lower - kEpsilon ||
        j.operational_position_upper > kOperationalScale * j.hard_position_upper + kEpsilon ||
        !(j.hard_velocity > 0) || !(j.operational_velocity > 0) || j.operational_velocity > j.hard_velocity + kEpsilon ||
        j.operational_velocity > kOperationalScale * j.hard_velocity + kEpsilon ||
        !(j.hard_effort > 0) || !(j.operational_effort > 0) ||
        j.operational_effort > kOperationalScale * j.hard_effort + kEpsilon) {
      Error(error, "invalid joint envelope"); return false;
    }
    seen[j.sdk_index] = true;
  }
  for (bool v : seen) if (!v) { Error(error, "joint indices incomplete"); return false; }
  return true;
}

SafetyProfile SafetyProfileForMotion(std::string_view motion_name) {
  if (motion_name == "pd_stand") return SafetyProfile::kPdStandRecovery;
  if (motion_name == "getup") return SafetyProfile::kGetup;
  return SafetyProfile::kPolicy;
}

void RequestSafetyProfile(SafetyProfile profile) {
  SafetyProfileLease lease;
  lease.profile = profile;
  lease.published_at_steady_ns = SteadyNowNs();
  data::VariantStore::GetInstance().Set(std::string(kSafetyProfileLeaseKey),
                                        lease);
}

void ReleaseSafetyProfile(SafetyProfile profile) {
  auto& store = data::VariantStore::GetInstance();
  const auto lease =
      store.GetCopy<SafetyProfileLease>(std::string(kSafetyProfileLeaseKey));
  if (lease && lease->profile == profile) {
    store.Remove(std::string(kSafetyProfileLeaseKey));
  }
}

SafetyProfile ResolveSafetyProfileForMotion(std::string_view motion_name,
                                            bool* downgraded) {
  if (downgraded) *downgraded = false;
  const SafetyProfile expected = SafetyProfileForMotion(motion_name);
  if (expected == SafetyProfile::kPolicy) return expected;

  const auto lease = data::VariantStore::GetInstance().GetCopy<SafetyProfileLease>(
      std::string(kSafetyProfileLeaseKey));
  const std::uint64_t now_ns = SteadyNowNs();
  const bool valid = lease && lease->profile == expected &&
                     lease->published_at_steady_ns != 0 &&
                     now_ns >= lease->published_at_steady_ns &&
                     now_ns - lease->published_at_steady_ns <=
                         kSafetyProfileLeaseFreshnessNs;
  if (!valid) {
    if (downgraded) *downgraded = true;
    return SafetyProfile::kPolicy;
  }
  return expected;
}

std::string_view SafetyProfileName(SafetyProfile profile) {
  switch (profile) {
    case SafetyProfile::kPolicy:
      return "policy";
    case SafetyProfile::kGetup:
      return "getup";
    case SafetyProfile::kPdStandRecovery:
      return "pd_stand_recovery";
  }
  return "unknown";
}

std::string_view SafetyStatusMessage(SafetyStatus status) {
  switch (status) {
    case SafetyStatus::kNormal:
      return "NORMAL：状态正常，命令在安全范围内";
    case SafetyStatus::kLimited:
      return "LIMITED：轻微越界或瞬时命令裁剪，仍允许受控运动";
    case SafetyStatus::kRecovery:
      return "RECOVERY：检测到可恢复越界，仅允许向安全区方向运动";
    case SafetyStatus::kFatal:
      return "FATAL：检测到不可恢复或数据/电机故障，已停止驱动输出";
  }
  return "FATAL：安全状态未知，已停止驱动输出";
}

void MotorEnableGuard::Reset() {
  was_ready_ = false;
  enable_wait_started_ns_ = 0;
}

MotorEnablePhase MotorEnableGuard::Update(bool enable_requested,
                                          bool driver_enable_confirmed,
                                          bool body_feedback_enabled,
                                          std::uint64_t now_ns) {
  if (!enable_requested) {
    Reset();
    return MotorEnablePhase::kIntentionallyDisabled;
  }

  if (driver_enable_confirmed && body_feedback_enabled) {
    was_ready_ = true;
    enable_wait_started_ns_ = 0;
    return MotorEnablePhase::kReady;
  }

  // A motor that drops out after the body was fully armed is not an arming
  // transient. Fail immediately so an active motion cannot continue on a
  // partial support set.
  if (was_ready_ || now_ns == 0) return MotorEnablePhase::kFault;

  if (enable_wait_started_ns_ == 0) enable_wait_started_ns_ = now_ns;
  if (now_ns < enable_wait_started_ns_ ||
      now_ns - enable_wait_started_ns_ > kMotorEnableGraceNs) {
    return MotorEnablePhase::kFault;
  }
  return MotorEnablePhase::kEnabling;
}

std::string_view MotorEnablePhaseMessage(MotorEnablePhase phase) {
  switch (phase) {
    case MotorEnablePhase::kReady:
      return "已使能";
    case MotorEnablePhase::kIntentionallyDisabled:
      return "软件请求断使能，安全输出保持为零";
    case MotorEnablePhase::kEnabling:
      return "电机使能中，安全输出保持为零";
    case MotorEnablePhase::kFault:
      return "电机使能失败或运行中掉使能";
  }
  return "未知电机使能状态";
}

SafetyStatus MoreSevereStatus(SafetyStatus lhs, SafetyStatus rhs) {
  return static_cast<int>(lhs) >= static_cast<int>(rhs) ? lhs : rhs;
}

std::string SafetyReasonSummary(std::uint32_t reason_mask) {
  if (reason_mask == kReasonNone) return "无";
  std::ostringstream text;
  bool first = true;
  const auto append = [&text, &first](std::string_view reason) {
    if (!first) text << ", ";
    first = false;
    text << reason;
  };
  if (reason_mask & kReasonNonFinite) append("非有限数值");
  if (reason_mask & kReasonHardPosition) append("绝对位置硬限");
  if (reason_mask & kReasonOperationalPosition) append("普通动作位置范围裁剪");
  if (reason_mask & kReasonRecoverablePosition) append("轻微位置越界可恢复");
  if (reason_mask & kReasonProfileAuthorization) {
    append("恢复档位授权缺失或过期");
  }
  if (reason_mask & kReasonMotorNotReady) append("电机尚未完成使能，暂缓输出");
  if (reason_mask & kReasonVelocity) append("速度限制");
  if (reason_mask & kReasonSlew) append("目标变化率限制");
  if (reason_mask & kReasonEffort) append("前馈力矩限制");
  if (reason_mask & kReasonPdEstimate) append("PD估算力矩限制");
  if (reason_mask & kReasonFullTorqueUnsupported) append("全力矩模式禁止");
  if (reason_mask & kReasonMotorFault) append("电机诊断故障");
  if (reason_mask & kReasonFrameFault) append("整帧故障");
  if (reason_mask & kReasonGain) append("控制增益非法");
  if (reason_mask & kReasonDeadline) append("数据超时");
  if (reason_mask & kReasonFeedbackJump) append("反馈跳变");
  if (reason_mask & kReasonFeedbackTracking) append("反馈跟踪异常");
  if (reason_mask & kReasonFeedbackMismatch) append("位置速度反馈不一致");
  return text.str();
}

HeadJointHealthMonitor::HeadJointHealthMonitor(const T800JointEnvelope& envelope) : envelope_(envelope) { Reset(); }
void HeadJointHealthMonitor::Reset() {
  snapshot_ = {};
  have_last_good_.fill(false);
  last_good_q_.fill(0.0);
  have_previous_sample_.fill(false);
  previous_sample_q_.fill(0.0);
  healthy_frames_.fill(0);
  tracking_fault_frames_.fill(0);
  velocity_mismatch_frames_.fill(0);
  recovery_direction_.fill(RecoveryDirection::kNone);
}
T800SafetySnapshot HeadJointHealthMonitor::Update(const T800JointState& raw, const T800MotorFlags& flags,
                                                   std::uint64_t motion_epoch, bool motor_flags_valid,
                                                   const T800CommandFrame* requested,
                                                   double sample_period,
                                                   bool disabled_motor_is_fault) {
  snapshot_.motion_epoch = motion_epoch;
  ++snapshot_.generation;
  snapshot_.status = SafetyStatus::kNormal;
  snapshot_.frame_fault = false;
  snapshot_.command_accepted = false;
  snapshot_.command_decision = SafetyDecision::kFault;
  snapshot_.reason_mask = kReasonNone;
  snapshot_.head_fault_mask.fill(0);
  snapshot_.affected_joint_mask.fill(0);
  snapshot_.recovery_direction.fill(RecoveryDirection::kNone);
  const bool sample_period_valid = std::isfinite(sample_period) && sample_period > 0.0 &&
                                   sample_period <= 0.02;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto& j = envelope_.joint(i);
    const JointHealth previous_health = snapshot_.health[i];
    const bool finite = std::isfinite(raw.q[i]) && std::isfinite(raw.qd[i]);
    RecoveryDirection recovery_direction = RecoveryDirection::kNone;
    const bool hard_position =
        finite && PositionOverrun(j, raw.q[i], &recovery_direction) > 0.0;
    const bool hard_velocity = finite && std::abs(raw.qd[i]) > j.hard_velocity + kEpsilon;
    const bool motor_fault =
        motor_flags_valid &&
        (flags.offline[i] != 0 || flags.error_code[i] != 0 ||
         (disabled_motor_is_fault && flags.enable[i] == 0));
    bool feedback_jump = false;
    bool feedback_tracking = false;
    bool feedback_mismatch = false;
    if (Head(i) && finite && sample_period_valid) {
      // The first finite sample after a failed head reading establishes a new
      // comparison baseline. Subsequent recovering samples are still checked,
      // so the 50-frame hysteresis cannot hide fresh jumps or mismatches.
      if (have_previous_sample_[i] &&
          previous_health != JointHealth::kFailed) {
        const double position_delta = std::abs(raw.q[i] - previous_sample_q_[i]);
        const double maximum_physical_delta =
            j.hard_velocity * sample_period * kHeadJumpVelocityMultiplier +
            kHeadJumpSlackRad;
        feedback_jump = position_delta > maximum_physical_delta;

        const bool mismatch_sample = position_delta <= kHeadStationaryDeltaRad &&
                                     std::abs(raw.qd[i]) > kHeadVelocityMismatchRadS;
        velocity_mismatch_frames_[i] =
            mismatch_sample ? std::min(velocity_mismatch_frames_[i] + 1,
                                       kHeadVelocityMismatchFaultFrames)
                            : 0;
        feedback_mismatch =
            velocity_mismatch_frames_[i] >= kHeadVelocityMismatchFaultFrames;
      }

      const bool command_is_finite =
          requested != nullptr && std::isfinite(requested->q_des[i]) &&
          std::isfinite(requested->kp[i]);
      const bool tracking_stall =
          previous_health == JointHealth::kHealthy && command_is_finite &&
          requested->kp[i] >= kHeadTrackingMinimumKp &&
          std::abs(requested->q_des[i] - raw.q[i]) > kHeadTrackingErrorRad &&
          std::abs(raw.qd[i]) < kHeadTrackingVelocityRadS;
      tracking_fault_frames_[i] =
          tracking_stall ? std::min(tracking_fault_frames_[i] + 1,
                                    kHeadTrackingFaultFrames)
                         : 0;
      feedback_tracking = tracking_fault_frames_[i] >= kHeadTrackingFaultFrames;

      previous_sample_q_[i] = raw.q[i];
      have_previous_sample_[i] = true;
    } else if (Head(i) && !finite) {
      tracking_fault_frames_[i] = 0;
      velocity_mismatch_frames_[i] = 0;
    }

    const bool recoverable_position =
        !Head(i) && finite && hard_position && !hard_velocity && !motor_fault &&
        RecoverablePositionOverrun(envelope_, j, raw.q[i], raw.qd[i],
                                   &recovery_direction);
    const bool bad = !finite || (hard_position && !recoverable_position) ||
                     hard_velocity || motor_fault || feedback_jump ||
                     feedback_tracking || feedback_mismatch;
    if (recoverable_position) {
      recovery_direction_[i] = recovery_direction;
      snapshot_.health[i] = JointHealth::kRecovering;
      snapshot_.affected_joint_mask[i] = 1;
      snapshot_.recovery_direction[i] = recovery_direction_[i];
      snapshot_.reason_mask |=
          kReasonHardPosition | kReasonRecoverablePosition;
      snapshot_.status = MoreSevereStatus(snapshot_.status,
                                          SafetyStatus::kRecovery);
      healthy_frames_[i] = 0;
    } else if (bad) {
      recovery_direction_[i] = RecoveryDirection::kNone;
      snapshot_.health[i] = Head(i) ? JointHealth::kFailed : JointHealth::kSuspect;
      snapshot_.affected_joint_mask[i] = 1;
      if (!finite) snapshot_.reason_mask |= kReasonNonFinite;
      if (hard_position) snapshot_.reason_mask |= kReasonHardPosition;
      if (hard_velocity) snapshot_.reason_mask |= kReasonVelocity;
      if (motor_fault) snapshot_.reason_mask |= kReasonMotorFault;
      if (feedback_jump) snapshot_.reason_mask |= kReasonFeedbackJump;
      if (feedback_tracking) snapshot_.reason_mask |= kReasonFeedbackTracking;
      if (feedback_mismatch) snapshot_.reason_mask |= kReasonFeedbackMismatch;
      if (!Head(i)) {
        snapshot_.reason_mask |= kReasonFrameFault;
        snapshot_.frame_fault = true;
        snapshot_.status = SafetyStatus::kFatal;
      } else {
        snapshot_.status = MoreSevereStatus(snapshot_.status,
                                            SafetyStatus::kLimited);
      }
      healthy_frames_[i] = 0;
    } else {
      if (Head(i)) {
        const bool recovering = previous_health == JointHealth::kFailed ||
                                previous_health == JointHealth::kRecovering;
        if (recovering) {
          ++healthy_frames_[i];
          snapshot_.health[i] = healthy_frames_[i] >= kHeadRecoveryHealthyFrames
                                    ? JointHealth::kHealthy
                                    : JointHealth::kRecovering;
        } else {
          snapshot_.health[i] = JointHealth::kHealthy;
          healthy_frames_[i] = 0;
        }
        if (snapshot_.health[i] == JointHealth::kHealthy || !have_last_good_[i]) {
          last_good_q_[i] = raw.q[i];
          have_last_good_[i] = true;
        }
        snapshot_.sanitized_state.q[i] = last_good_q_[i];
        snapshot_.sanitized_state.qd[i] =
            snapshot_.health[i] == JointHealth::kHealthy ? raw.qd[i] : 0.0;
      } else {
        if (previous_health == JointHealth::kRecovering) {
          ++healthy_frames_[i];
          snapshot_.health[i] =
              healthy_frames_[i] >= envelope_.recovery_limits().healthy_frames
                  ? JointHealth::kHealthy
                  : JointHealth::kRecovering;
          if (snapshot_.health[i] == JointHealth::kHealthy) {
            recovery_direction_[i] = RecoveryDirection::kNone;
          }
        } else {
          snapshot_.health[i] = JointHealth::kHealthy;
          healthy_frames_[i] = 0;
          recovery_direction_[i] = RecoveryDirection::kNone;
        }
        if (snapshot_.health[i] == JointHealth::kRecovering) {
          snapshot_.affected_joint_mask[i] = 1;
          snapshot_.recovery_direction[i] = recovery_direction_[i];
          snapshot_.reason_mask |= kReasonRecoverablePosition;
          snapshot_.status = MoreSevereStatus(snapshot_.status,
                                              SafetyStatus::kRecovery);
        } else {
          last_good_q_[i] = raw.q[i];
          have_last_good_[i] = true;
        }
      }
    }
    if (Head(i) && (snapshot_.health[i] != JointHealth::kHealthy || !have_last_good_[i])) {
      snapshot_.head_fault_mask[i] = 1;
      snapshot_.affected_joint_mask[i] = 1;
      snapshot_.status = MoreSevereStatus(snapshot_.status,
                                          SafetyStatus::kLimited);
      snapshot_.sanitized_state.qd[i] = 0.0;
      if (have_last_good_[i]) snapshot_.sanitized_state.q[i] = last_good_q_[i];
      else snapshot_.sanitized_state.q[i] = std::clamp(0.0, j.operational_position_lower, j.operational_position_upper);
    }
    if (!Head(i)) {
      snapshot_.sanitized_state.q[i] =
          bad ? (have_last_good_[i]
                     ? last_good_q_[i]
                     : std::clamp(0.0, j.operational_position_lower,
                                  j.operational_position_upper))
              : raw.q[i];
      snapshot_.sanitized_state.qd[i] = bad ? 0.0 : raw.qd[i];
    }
    snapshot_.sanitized_state.tau[i] = std::isfinite(raw.tau[i]) ? raw.tau[i] : 0.0;
  }
  if (snapshot_.frame_fault) snapshot_.status = SafetyStatus::kFatal;
  return snapshot_;
}

JointCommandSafetyGate::JointCommandSafetyGate(const T800JointEnvelope& envelope) : envelope_(envelope) { Reset(); }
void JointCommandSafetyGate::Reset() {
  last_safe_ = {};
  last_profile_ = SafetyProfile::kPolicy;
  have_last_safe_ = false;
}
GateResult JointCommandSafetyGate::Evaluate(const T800CommandFrame& requested, const T800JointState& measured,
                                             double producer_dt,
                                             SafetyProfile profile,
                                             const std::array<RecoveryDirection, kJointCount>*
                                                 required_recovery_direction) {
  GateResult result;
  result.profile = profile;
  result.frame = have_last_safe_ ? last_safe_ : T800CommandFrame{};
  if (!std::isfinite(producer_dt) || producer_dt <= 0.0 || producer_dt > 0.02) {
    result.reason_mask |= kReasonDeadline;
    return result;
  }
  if (requested.full_torque_enabled) {
    result.reason_mask |= kReasonFullTorqueUnsupported;
    return result;
  }
  if (!Finite(requested.q_des) || !Finite(requested.qd_des) || !Finite(requested.kp) ||
      !Finite(requested.kd) || !Finite(requested.tau_ff) || !Finite(requested.full_torque) ||
      !Finite(measured.q) || !Finite(measured.qd)) {
    result.reason_mask |= kReasonNonFinite;
    return result;
  }
  SafetyStatus accepted_status = SafetyStatus::kNormal;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (requested.kp[i] < 0.0 || requested.kd[i] < 0.0) {
      result.reason_mask |= kReasonGain;
      result.affected_joint_mask[i] = 1;
      return result;
    }
    const auto& j = envelope_.joint(i);
    RecoveryDirection direction = RecoveryDirection::kNone;
    const double hard_overrun = PositionOverrun(j, measured.q[i], &direction);
    const bool recoverable_overrun =
        hard_overrun > 0.0 &&
        RecoverablePositionOverrun(envelope_, j, measured.q[i], measured.qd[i],
                                   &direction);
    if (hard_overrun > 0.0 && !recoverable_overrun) {
      result.reason_mask |= kReasonHardPosition;
      result.affected_joint_mask[i] = 1;
      return result;
    }
    if (recoverable_overrun) {
      accepted_status = SafetyStatus::kRecovery;
      result.reason_mask |= kReasonHardPosition | kReasonRecoverablePosition;
      result.affected_joint_mask[i] = 1;
      result.recovery_direction[i] = direction;
    } else if (required_recovery_direction != nullptr &&
               (*required_recovery_direction)[i] != RecoveryDirection::kNone) {
      accepted_status = SafetyStatus::kRecovery;
      result.reason_mask |= kReasonRecoverablePosition;
      result.affected_joint_mask[i] = 1;
      result.recovery_direction[i] = (*required_recovery_direction)[i];
    }
    if (std::abs(measured.qd[i]) > j.hard_velocity + kEpsilon) {
      result.reason_mask |= kReasonVelocity;
      result.affected_joint_mask[i] = 1;
      return result;
    }
  }
  bool clipped = false;
  const bool profile_changed = !have_last_safe_ || profile != last_profile_;
  result.frame = requested;
  result.frame.full_torque_enabled = false;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto& j = envelope_.joint(i);
    const double position_lower = PositionLower(j, profile);
    const double position_upper = PositionUpper(j, profile);
    RecoveryDirection command_direction = result.recovery_direction[i];
    const bool hard_limit_recovery =
        command_direction != RecoveryDirection::kNone;
    if (command_direction == RecoveryDirection::kNone) {
      if (measured.q[i] < position_lower) {
        command_direction = RecoveryDirection::kIncrease;
      } else if (measured.q[i] > position_upper) {
        command_direction = RecoveryDirection::kDecrease;
      }
    }
    if (command_direction != RecoveryDirection::kNone &&
        !hard_limit_recovery) {
      result.reason_mask |= kReasonOperationalPosition;
      result.affected_joint_mask[i] = 1;
      clipped = true;
    }

    const double bounded_q =
        std::clamp(result.frame.q_des[i], position_lower, position_upper);
    if (bounded_q != result.frame.q_des[i]) {
      result.reason_mask |= UsesHardPositionEnvelope(profile)
                                ? kReasonHardPosition
                                : kReasonOperationalPosition;
      clipped = true;
      result.affected_joint_mask[i] = 1;
    }
    const double max_delta = j.operational_velocity * producer_dt;
    const bool last_safe_in_profile =
        have_last_safe_ && last_safe_.q_des[i] >= position_lower - kEpsilon &&
        last_safe_.q_des[i] <= position_upper + kEpsilon;
    const double slew_reference =
        profile_changed || command_direction != RecoveryDirection::kNone ||
                !last_safe_in_profile
            ? measured.q[i]
            : last_safe_.q_des[i];
    double allowed_q_lower = std::max(position_lower, slew_reference - max_delta);
    double allowed_q_upper = std::min(position_upper, slew_reference + max_delta);
    if (command_direction == RecoveryDirection::kIncrease) {
      allowed_q_lower = slew_reference;
      allowed_q_upper = slew_reference + max_delta;
      result.recovery_direction[i] = command_direction;
    } else if (command_direction == RecoveryDirection::kDecrease) {
      allowed_q_lower = slew_reference - max_delta;
      allowed_q_upper = slew_reference;
      result.recovery_direction[i] = command_direction;
    }
    if (allowed_q_lower > allowed_q_upper + kEpsilon) {
      result.reason_mask |= kReasonHardPosition;
      result.affected_joint_mask[i] = 1;
      return result;
    }
    result.frame.q_des[i] =
        std::clamp(bounded_q, allowed_q_lower, allowed_q_upper);
    if (std::abs(result.frame.q_des[i] - bounded_q) > kEpsilon) {
      result.reason_mask |= kReasonSlew;
      result.affected_joint_mask[i] = 1;
      clipped = true;
    }

    double allowed_qd_lower = -j.operational_velocity;
    double allowed_qd_upper = j.operational_velocity;
    if (command_direction == RecoveryDirection::kIncrease) {
      allowed_qd_lower = 0.0;
    } else if (command_direction == RecoveryDirection::kDecrease) {
      allowed_qd_upper = 0.0;
    }
    const double qd = std::clamp(result.frame.qd_des[i], allowed_qd_lower,
                                 allowed_qd_upper);
    if (qd != result.frame.qd_des[i]) {
      result.reason_mask |= kReasonVelocity;
      clipped = true;
      result.affected_joint_mask[i] = 1;
      result.frame.qd_des[i] = qd;
    }
    result.frame.tau_ff[i] =
        std::clamp(result.frame.tau_ff[i], -j.operational_effort, j.operational_effort);
    if (result.frame.tau_ff[i] != requested.tau_ff[i]) {
      result.reason_mask |= kReasonEffort;
      clipped = true;
      result.affected_joint_mask[i] = 1;
    }
    if (command_direction == RecoveryDirection::kIncrease &&
        result.frame.tau_ff[i] < 0.0) {
      result.frame.tau_ff[i] = 0.0;
      result.reason_mask |= kReasonEffort;
      result.affected_joint_mask[i] = 1;
      clipped = true;
    } else if (command_direction == RecoveryDirection::kDecrease &&
               result.frame.tau_ff[i] > 0.0) {
      result.frame.tau_ff[i] = 0.0;
      result.reason_mask |= kReasonEffort;
      result.affected_joint_mask[i] = 1;
      clipped = true;
    }

    const double non_position_effort =
        result.frame.kd[i] * (result.frame.qd_des[i] - measured.qd[i]) +
        result.frame.tau_ff[i];
    if (!std::isfinite(non_position_effort)) {
      result.reason_mask |= kReasonPdEstimate;
      result.affected_joint_mask[i] = 1;
      return result;
    }
    if (result.frame.kp[i] > kEpsilon) {
      const double pd_q_lower =
          measured.q[i] +
          (-j.operational_effort - non_position_effort) / result.frame.kp[i];
      const double pd_q_upper =
          measured.q[i] +
          (j.operational_effort - non_position_effort) / result.frame.kp[i];
      const double safe_q_lower = std::max(allowed_q_lower, pd_q_lower);
      const double safe_q_upper = std::min(allowed_q_upper, pd_q_upper);
      if (safe_q_lower > safe_q_upper + kEpsilon) {
        result.reason_mask |= kReasonPdEstimate;
        result.affected_joint_mask[i] = 1;
        return result;
      }
      const double effort_limited_q =
          std::clamp(result.frame.q_des[i], safe_q_lower, safe_q_upper);
      if (std::abs(effort_limited_q - result.frame.q_des[i]) > kEpsilon) {
        result.frame.q_des[i] = effort_limited_q;
        result.reason_mask |= kReasonPdEstimate;
        result.affected_joint_mask[i] = 1;
        clipped = true;
      }
    } else if (result.frame.kd[i] > kEpsilon) {
      const double pd_qd_lower =
          measured.qd[i] +
          (-j.operational_effort - result.frame.tau_ff[i]) /
              result.frame.kd[i];
      const double pd_qd_upper =
          measured.qd[i] +
          (j.operational_effort - result.frame.tau_ff[i]) /
              result.frame.kd[i];
      const double safe_qd_lower = std::max(allowed_qd_lower, pd_qd_lower);
      const double safe_qd_upper = std::min(allowed_qd_upper, pd_qd_upper);
      if (safe_qd_lower > safe_qd_upper + kEpsilon) {
        result.reason_mask |= kReasonPdEstimate;
        result.affected_joint_mask[i] = 1;
        return result;
      }
      const double effort_limited_qd =
          std::clamp(result.frame.qd_des[i], safe_qd_lower, safe_qd_upper);
      if (std::abs(effort_limited_qd - result.frame.qd_des[i]) > kEpsilon) {
        result.frame.qd_des[i] = effort_limited_qd;
        result.reason_mask |= kReasonPdEstimate;
        result.affected_joint_mask[i] = 1;
        clipped = true;
      }
    }
    const double pd = result.frame.kp[i] *
                          (result.frame.q_des[i] - measured.q[i]) +
                      result.frame.kd[i] *
                          (result.frame.qd_des[i] - measured.qd[i]) +
                      result.frame.tau_ff[i];
    if (!std::isfinite(pd) ||
        std::abs(pd) > j.operational_effort + kEpsilon) {
      result.reason_mask |= kReasonPdEstimate;
      result.affected_joint_mask[i] = 1;
      return result;
    }
  }
  result.accepted = true;
  if (accepted_status == SafetyStatus::kRecovery) {
    result.status = SafetyStatus::kRecovery;
    result.decision = SafetyDecision::kRecovery;
  } else {
    result.status = clipped ? SafetyStatus::kLimited : SafetyStatus::kNormal;
    result.decision =
        clipped ? SafetyDecision::kClipped : SafetyDecision::kAccepted;
  }
  last_safe_ = result.frame;
  last_profile_ = profile;
  have_last_safe_ = true;
  return result;
}

GateResult MotorCommandSafetyGate::Evaluate(const T800CommandFrame& requested,
                                            const T800JointState& measured,
                                            SafetyProfile profile,
                                            const std::array<RecoveryDirection, kJointCount>*
                                                required_recovery_direction) const {
  GateResult result;
  result.profile = profile;
  result.frame = requested;
  if (requested.full_torque_enabled) {
    result.reason_mask |= kReasonFullTorqueUnsupported;
    return result;
  }
  if (!Finite(requested.q_des) || !Finite(requested.qd_des) || !Finite(requested.kp) ||
      !Finite(requested.kd) || !Finite(requested.tau_ff) ||
      !Finite(requested.full_torque) || !Finite(measured.q) ||
      !Finite(measured.qd) || !Finite(measured.tau)) {
    result.reason_mask |= kReasonNonFinite;
    return result;
  }
  SafetyStatus accepted_status = SafetyStatus::kNormal;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (requested.kp[i] < 0.0 || requested.kd[i] < 0.0) {
      result.reason_mask |= kReasonGain;
      result.affected_joint_mask[i] = 1;
      return result;
    }
    if (ParallelAnkle(i)) {
      continue;
    }
    const auto& joint = envelope_.joint(i);
    RecoveryDirection hard_direction = RecoveryDirection::kNone;
    const double hard_overrun =
        PositionOverrun(joint, measured.q[i], &hard_direction);
    const bool recoverable_overrun =
        hard_overrun > 0.0 && RecoverablePositionOverrun(
                                  envelope_, joint, measured.q[i], measured.qd[i],
                                  &hard_direction);
    if (hard_overrun > 0.0 && !recoverable_overrun) {
      result.reason_mask |= kReasonHardPosition;
      result.affected_joint_mask[i] = 1;
      return result;
    }
    const RecoveryDirection required_direction =
        required_recovery_direction != nullptr
            ? (*required_recovery_direction)[i]
            : RecoveryDirection::kNone;
    const bool recovery_required =
        recoverable_overrun || required_direction != RecoveryDirection::kNone;
    if (recovery_required) {
      accepted_status = SafetyStatus::kRecovery;
      result.reason_mask |= kReasonRecoverablePosition;
      if (recoverable_overrun) result.reason_mask |= kReasonHardPosition;
      result.affected_joint_mask[i] = 1;
      result.recovery_direction[i] =
          recoverable_overrun ? hard_direction : required_direction;
    }
    if (std::abs(measured.qd[i]) > joint.hard_velocity + kEpsilon) {
      result.reason_mask |= kReasonVelocity;
      result.affected_joint_mask[i] = 1;
      return result;
    }

    const double position_lower = PositionLower(joint, profile);
    const double position_upper = PositionUpper(joint, profile);
    RecoveryDirection command_direction =
        recoverable_overrun ? hard_direction : required_direction;
    if (command_direction == RecoveryDirection::kNone) {
      if (measured.q[i] < position_lower) {
        command_direction = RecoveryDirection::kIncrease;
      } else if (measured.q[i] > position_upper) {
        command_direction = RecoveryDirection::kDecrease;
      }
    }
    const bool command_inside =
        requested.q_des[i] >= position_lower - kEpsilon &&
        requested.q_des[i] <= position_upper + kEpsilon;
    const bool command_recovering_inward =
        (command_direction == RecoveryDirection::kIncrease &&
         requested.q_des[i] >= measured.q[i] - kEpsilon &&
         requested.q_des[i] <= position_lower + kEpsilon) ||
        (command_direction == RecoveryDirection::kDecrease &&
         requested.q_des[i] <= measured.q[i] + kEpsilon &&
         requested.q_des[i] >= position_upper - kEpsilon);
    const bool command_position_inward_or_neutral =
        command_direction == RecoveryDirection::kNone ||
        (command_direction == RecoveryDirection::kIncrease &&
         requested.q_des[i] >= measured.q[i] - kEpsilon) ||
        (command_direction == RecoveryDirection::kDecrease &&
         requested.q_des[i] <= measured.q[i] + kEpsilon);
    if ((!command_inside && !command_recovering_inward) ||
        !command_position_inward_or_neutral) {
      result.reason_mask |= recovery_required || UsesHardPositionEnvelope(profile)
                                ? kReasonHardPosition
                                : kReasonOperationalPosition;
      result.affected_joint_mask[i] = 1;
      return result;
    }
    if (std::abs(requested.qd_des[i]) > joint.operational_velocity + kEpsilon) {
      result.reason_mask |= kReasonVelocity;
      result.affected_joint_mask[i] = 1;
      return result;
    }
    if ((command_direction == RecoveryDirection::kIncrease &&
         requested.qd_des[i] < -kEpsilon) ||
        (command_direction == RecoveryDirection::kDecrease &&
         requested.qd_des[i] > kEpsilon)) {
      result.reason_mask |= kReasonVelocity;
      result.affected_joint_mask[i] = 1;
      return result;
    }
    const double effort_limit = joint.operational_effort;
    if (std::abs(requested.tau_ff[i]) > effort_limit + kEpsilon) {
      result.reason_mask |= kReasonEffort;
      result.affected_joint_mask[i] = 1;
      return result;
    }
    if ((command_direction == RecoveryDirection::kIncrease &&
         requested.tau_ff[i] < -kEpsilon) ||
        (command_direction == RecoveryDirection::kDecrease &&
         requested.tau_ff[i] > kEpsilon)) {
      result.reason_mask |= kReasonEffort;
      result.affected_joint_mask[i] = 1;
      return result;
    }
    const double pd_effort =
        requested.kp[i] * (requested.q_des[i] - measured.q[i]) +
        requested.kd[i] * (requested.qd_des[i] - measured.qd[i]) +
        requested.tau_ff[i];
    if (!std::isfinite(pd_effort) || std::abs(pd_effort) > effort_limit + kEpsilon) {
      result.reason_mask |= kReasonPdEstimate;
      result.affected_joint_mask[i] = 1;
      return result;
    }
    if (command_direction != RecoveryDirection::kNone) {
      result.recovery_direction[i] = command_direction;
      result.affected_joint_mask[i] = 1;
      if (!recovery_required) {
        accepted_status = MoreSevereStatus(accepted_status,
                                           SafetyStatus::kLimited);
        result.reason_mask |= kReasonOperationalPosition;
      }
    }
  }
  result.accepted = true;
  result.status = accepted_status;
  if (result.status == SafetyStatus::kRecovery) {
    result.decision = SafetyDecision::kRecovery;
  } else if (result.status == SafetyStatus::kLimited) {
    result.decision = SafetyDecision::kClipped;
  } else {
    result.status = SafetyStatus::kNormal;
    result.decision = SafetyDecision::kAccepted;
  }
  return result;
}

bool GetT800SanitizedState(Eigen::VectorXd* q, Eigen::VectorXd* qd, T800SafetySnapshot* snapshot) {
  if (q == nullptr || qd == nullptr || q->size() != static_cast<Eigen::Index>(kJointCount) ||
      qd->size() != static_cast<Eigen::Index>(kJointCount)) {
    if (snapshot) {
      *snapshot = {};
      snapshot->status = SafetyStatus::kFatal;
      snapshot->frame_fault = true;
      snapshot->reason_mask = kReasonFrameFault;
    }
    return false;
  }

  const auto copy =
      data::VariantStore::GetInstance().GetCopy<T800SafetySnapshot>("safety/t800_snapshot");
  if (!copy) {
    T800SafetySnapshot local = SanitizeRawState(*q, *qd);
    local.status = SafetyStatus::kFatal;
    local.frame_fault = true;
    local.reason_mask |= kReasonDeadline | kReasonFrameFault;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      (*q)[static_cast<Eigen::Index>(i)] = local.sanitized_state.q[i];
      (*qd)[static_cast<Eigen::Index>(i)] = local.sanitized_state.qd[i];
    }
    if (snapshot) *snapshot = local;
    return false;
  }

  const std::uint64_t now_ns = SteadyNowNs();
  const bool fresh = copy->published_at_steady_ns != 0 &&
                     now_ns >= copy->published_at_steady_ns &&
                     now_ns - copy->published_at_steady_ns <= kSnapshotFreshnessLimitNs;
  if (!fresh) {
    T800SafetySnapshot local = SanitizeRawState(*q, *qd);
    local.status = SafetyStatus::kFatal;
    local.frame_fault = true;
    local.reason_mask |= kReasonDeadline | kReasonFrameFault;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      (*q)[static_cast<Eigen::Index>(i)] = local.sanitized_state.q[i];
      (*qd)[static_cast<Eigen::Index>(i)] = local.sanitized_state.qd[i];
    }
    if (snapshot) *snapshot = local;
    return false;
  }

  if (snapshot) *snapshot = *copy;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    (*q)[static_cast<Eigen::Index>(i)] =
        std::isfinite(copy->sanitized_state.q[i]) ? copy->sanitized_state.q[i] : 0.0;
    (*qd)[static_cast<Eigen::Index>(i)] =
        std::isfinite(copy->sanitized_state.qd[i]) ? copy->sanitized_state.qd[i] : 0.0;
  }
  return copy->status != SafetyStatus::kFatal && !copy->frame_fault;
}

void MaskFailedHeadActions(const T800SafetySnapshot& snapshot,
                           const Eigen::VectorXi& action_to_joint,
                           Eigen::VectorXd* actions) {
  if (actions == nullptr || actions->size() != action_to_joint.size()) {
    return;
  }
  for (Eigen::Index action_index = 0; action_index < action_to_joint.size(); ++action_index) {
    const int joint_index = action_to_joint[action_index];
    if (joint_index == static_cast<int>(kHeadPitchIndex) &&
        snapshot.head_fault_mask[kHeadPitchIndex] != 0) {
      (*actions)[action_index] = 0.0;
    } else if (joint_index == static_cast<int>(kHeadYawIndex) &&
               snapshot.head_fault_mask[kHeadYawIndex] != 0) {
      (*actions)[action_index] = 0.0;
    }
  }
}

bool IsT800Model(const data::ModelParam& model) {
  if (model.num_total_joints != static_cast<int>(kJointCount)) return false;
  const auto pitch = model.joint_id_in_total_limb.find("J23_HEAD_PITCH");
  const auto yaw = model.joint_id_in_total_limb.find("J24_HEAD_YAW");
  return pitch != model.joint_id_in_total_limb.end() &&
         yaw != model.joint_id_in_total_limb.end() &&
         pitch->second == static_cast<int>(kHeadPitchIndex) &&
         yaw->second == static_cast<int>(kHeadYawIndex);
}

}  // namespace runner::t800_safety

namespace runner {
namespace {

constexpr double kModelLimitTolerance = 1e-4;
constexpr double kResidentPeriod = 0.002;
constexpr double kMotorCommandTolerance = 1e-9;
constexpr std::uint64_t kPersistentStatusReportFrames = 500;

bool HasExpectedSize(const Eigen::VectorXd& values) {
  return values.size() == static_cast<Eigen::Index>(t800_safety::kJointCount);
}

bool IsFullTorqueRequested(const Eigen::VectorXd& torque) {
  if (!torque.allFinite()) return true;
  return (torque.array().abs() > 1e-9).any();
}

std::string CurrentMotionName(const std::shared_ptr<data::DataStore>& data_store) {
  if (!data_store) return "<unknown>";
  const auto motion = data_store->current_motion_task_name.Get();
  return motion && !motion->empty() ? *motion : "<unknown>";
}

std::string FormatAffectedJoints(
    const t800_safety::T800SafetySnapshot& snapshot,
    const t800_safety::T800JointEnvelope& envelope,
    const t800_safety::T800JointState& measured,
    const t800_safety::T800CommandFrame& command) {
  std::ostringstream text;
  text << std::fixed << std::setprecision(6);
  bool first = true;
  std::size_t affected_count = 0;
  std::size_t shown_count = 0;
  constexpr std::size_t kMaximumShownJoints = 6;
  for (std::size_t i = 0; i < t800_safety::kJointCount; ++i) {
    if (snapshot.affected_joint_mask[i] == 0) continue;
    ++affected_count;
    if (shown_count >= kMaximumShownJoints) continue;
    if (!first) text << "; ";
    first = false;
    ++shown_count;
    text << envelope.joint(i).sdk_name << "(q=" << measured.q[i]
         << ", q_des=" << command.q_des[i];
    if (snapshot.recovery_direction[i] ==
        t800_safety::RecoveryDirection::kIncrease) {
      text << ", direction=增大至安全区";
    } else if (snapshot.recovery_direction[i] ==
               t800_safety::RecoveryDirection::kDecrease) {
      text << ", direction=减小至安全区";
    }
    text << ')';
  }
  if (affected_count > shown_count) {
    text << "; 另有" << affected_count - shown_count << "个关节";
  }
  return first ? "无" : text.str();
}

}  // namespace

bool T800SafetyRunner::Initialize() {
  const auto fail_closed = [this]() {
    if (data_store_) data_store_->joint_info.SetZeroCommand();
    t800_safety::T800SafetySnapshot invalid;
    invalid.status = t800_safety::SafetyStatus::kFatal;
    invalid.frame_fault = true;
    invalid.command_accepted = false;
    invalid.command_decision = t800_safety::SafetyDecision::kFault;
    invalid.reason_mask = t800_safety::kReasonFrameFault;
    invalid.published_at_steady_ns = t800_safety::SteadyNowNs();
    data::VariantStore::GetInstance().Set("safety/t800_snapshot", invalid);
    return false;
  };

  if (!data_store_ || !data_store_->model_param ||
      !t800_safety::IsT800Model(*data_store_->model_param)) {
    LOG(ERROR) << "[T800SafetyRunner] T800 requires exactly "
               << t800_safety::kJointCount << " joints";
    return fail_closed();
  }

  const std::string contract_path = common::PathJoin(
      common::GlobalPathManager::GetInstance().GetConfigPath(),
      "safety/t800_safety_contract.json");
  std::string error;
  if (!t800_safety::T800JointEnvelope::Load(contract_path, &envelope_, &error)) {
    LOG(ERROR) << "[T800SafetyRunner] Failed to load " << contract_path << ": " << error;
    return fail_closed();
  }

  // JointInfo::GetLimit writes into caller-owned storage without resizing it.
  Eigen::VectorXd upper(t800_safety::kJointCount);
  Eigen::VectorXd lower(t800_safety::kJointCount);
  Eigen::VectorXd velocity(t800_safety::kJointCount);
  Eigen::VectorXd effort(t800_safety::kJointCount);
  data_store_->joint_info.GetLimit(upper, lower, velocity, effort);
  if (!HasExpectedSize(upper) || !HasExpectedSize(lower) || !HasExpectedSize(velocity) ||
      !HasExpectedSize(effort) || !upper.allFinite() || !lower.allFinite() ||
      !velocity.allFinite() || !effort.allFinite()) {
    LOG(ERROR) << "[T800SafetyRunner] Model limits are missing or invalid";
    return fail_closed();
  }

  for (std::size_t i = 0; i < t800_safety::kJointCount; ++i) {
    const auto& joint = envelope_.joint(i);
    const auto model_joint = data_store_->model_param->joint_id_in_total_limb.find(joint.sdk_name);
    const bool mapping_matches =
        model_joint != data_store_->model_param->joint_id_in_total_limb.end() &&
        model_joint->second == static_cast<int>(i);
    const bool limits_match =
        std::abs(lower[static_cast<Eigen::Index>(i)] - joint.hard_position_lower) <=
            kModelLimitTolerance &&
        std::abs(upper[static_cast<Eigen::Index>(i)] - joint.hard_position_upper) <=
            kModelLimitTolerance &&
        std::abs(velocity[static_cast<Eigen::Index>(i)] - joint.hard_velocity) <=
            kModelLimitTolerance &&
        std::abs(effort[static_cast<Eigen::Index>(i)] - joint.hard_effort) <=
            kModelLimitTolerance;
    if (!mapping_matches || !limits_match) {
      LOG(ERROR) << "[T800SafetyRunner] Contract/model mismatch at SDK joint " << i
                 << " (" << joint.sdk_name << ")";
      return fail_closed();
    }
  }

  health_ = std::make_unique<t800_safety::HeadJointHealthMonitor>(envelope_);
  gate_ = std::make_unique<t800_safety::JointCommandSafetyGate>(envelope_);
  motor_enable_guard_.Reset();
  motor_debug_subscriber_ =
      data::VariantStore::GetInstance().CreateSubscriber<data::MotorDebug>(
          "hardware/motor_debug");
  last_motor_debug_sample_.reset();
  last_motor_debug_update_ns_ = 0;
  have_reported_status_ = false;
  last_reported_status_ = t800_safety::SafetyStatus::kNormal;
  last_reported_profile_ = t800_safety::SafetyProfile::kPolicy;
  last_reported_motor_enable_phase_ =
      t800_safety::MotorEnablePhase::kReady;
  last_reported_reason_mask_ = t800_safety::kReasonNone;
  last_reported_motion_.clear();
  status_frames_since_report_ = 0;
  initialized_ = true;
  return true;
}

bool T800SafetyRunner::Enter() { return initialized_ || Initialize(); }

void T800SafetyRunner::Run() {
  if ((!initialized_ && !Initialize()) || !data_store_ || !health_ || !gate_) {
    if (data_store_) data_store_->joint_info.SetZeroCommand();
    t800_safety::T800SafetySnapshot invalid;
    invalid.status = t800_safety::SafetyStatus::kFatal;
    invalid.frame_fault = true;
    invalid.command_accepted = false;
    invalid.command_decision = t800_safety::SafetyDecision::kFault;
    invalid.reason_mask = t800_safety::kReasonFrameFault;
    invalid.published_at_steady_ns = t800_safety::SteadyNowNs();
    data::VariantStore::GetInstance().Set("safety/t800_snapshot", invalid);
    return;
  }

  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd tau;
  Eigen::VectorXd q_des;
  Eigen::VectorXd qd_des;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
  Eigen::VectorXd tau_ff;
  Eigen::VectorXd full_torque;
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q);
  data_store_->joint_info.GetState(data::JointInfoType::kVelocity, qd);
  data_store_->joint_info.GetState(data::JointInfoType::kTorque, tau);
  data_store_->joint_info.GetCommand(data::JointInfoType::kPosition, q_des);
  data_store_->joint_info.GetCommand(data::JointInfoType::kVelocity, qd_des);
  data_store_->joint_info.GetCommand(data::JointInfoType::kStiffness, kp);
  data_store_->joint_info.GetCommand(data::JointInfoType::kDamping, kd);
  data_store_->joint_info.GetCommand(data::JointInfoType::kFeedForwardTorque, tau_ff);
  data_store_->joint_info.GetCommand(data::JointInfoType::kTorque, full_torque);

  if (!HasExpectedSize(q) || !HasExpectedSize(qd) || !HasExpectedSize(tau) ||
      !HasExpectedSize(q_des) || !HasExpectedSize(qd_des) || !HasExpectedSize(kp) ||
      !HasExpectedSize(kd) || !HasExpectedSize(tau_ff) || !HasExpectedSize(full_torque)) {
    LOG_EVERY_N(ERROR, 100) << "[T800SafetyRunner] Joint state/command dimension mismatch";
    data_store_->joint_info.SetZeroCommand();
    t800_safety::T800SafetySnapshot invalid;
    invalid.status = t800_safety::SafetyStatus::kFatal;
    invalid.frame_fault = true;
    invalid.reason_mask = t800_safety::kReasonFrameFault;
    invalid.published_at_steady_ns = t800_safety::SteadyNowNs();
    data::VariantStore::GetInstance().Set("safety/t800_snapshot", invalid);
    return;
  }

  t800_safety::T800JointState state;
  t800_safety::T800CommandFrame command;
  const std::string motion_name = CurrentMotionName(data_store_);
  bool profile_downgraded = false;
  const t800_safety::SafetyProfile profile =
      t800_safety::ResolveSafetyProfileForMotion(motion_name,
                                                 &profile_downgraded);
  for (std::size_t i = 0; i < t800_safety::kJointCount; ++i) {
    const Eigen::Index index = static_cast<Eigen::Index>(i);
    state.q[i] = q[index];
    state.qd[i] = qd[index];
    state.tau[i] = tau[index];
    command.q_des[i] = q_des[index];
    command.qd_des[i] = qd_des[index];
    command.kp[i] = kp[index];
    command.kd[i] = kd[index];
    command.tau_ff[i] = tau_ff[index];
    command.full_torque[i] = full_torque[index];
  }
  command.full_torque_enabled = IsFullTorqueRequested(full_torque);

  t800_safety::T800MotorFlags motor_flags;
  auto& variant_store = data::VariantStore::GetInstance();
  const auto motor_debug = motor_debug_subscriber_.Get();
  const std::uint64_t now_ns = t800_safety::SteadyNowNs();
  if (motor_debug &&
      (!last_motor_debug_sample_ ||
       motor_debug.get() != last_motor_debug_sample_.get())) {
    // GuardedData::Set replaces its shared sample. Retaining the previous
    // shared_ptr prevents allocator address reuse from looking like an update.
    last_motor_debug_sample_ = motor_debug;
    last_motor_debug_update_ns_ = now_ns;
  }
  const bool motor_debug_layout_valid =
      motor_debug && motor_debug->offline.size() == t800_safety::kJointCount &&
      motor_debug->enable.size() == t800_safety::kJointCount &&
      motor_debug->error_code.size() == t800_safety::kJointCount;
  const bool motor_debug_fresh =
      last_motor_debug_update_ns_ != 0 && now_ns >= last_motor_debug_update_ns_ &&
      now_ns - last_motor_debug_update_ns_ <=
          t800_safety::kMotorDebugFreshnessLimitNs;
  const bool require_motor_debug = !common::IsInMujoco();
  const bool motor_flags_valid =
      require_motor_debug && motor_debug_layout_valid && motor_debug_fresh;
  if (motor_flags_valid) {
    for (std::size_t i = 0; i < t800_safety::kJointCount; ++i) {
      motor_flags.offline[i] = motor_debug->offline[i];
      motor_flags.enable[i] = motor_debug->enable[i];
      motor_flags.error_code[i] = motor_debug->error_code[i];
    }
  }

  bool body_feedback_enabled = motor_flags_valid;
  if (motor_flags_valid) {
    for (std::size_t i = 0; i < t800_safety::kHeadPitchIndex; ++i) {
      body_feedback_enabled =
          body_feedback_enabled && motor_flags.enable[i] != 0;
    }
  }
  const bool motor_enable_requested =
      data_store_->enable_or_disable_motor.load();
  const bool driver_enable_confirmed =
      data_store_->enabled_motor_state.load();
  const t800_safety::MotorEnablePhase motor_enable_phase =
      require_motor_debug
          ? motor_enable_guard_.Update(motor_enable_requested,
                                       driver_enable_confirmed,
                                       body_feedback_enabled, now_ns)
          : t800_safety::MotorEnablePhase::kReady;
  const bool motor_not_ready =
      motor_enable_phase ==
          t800_safety::MotorEnablePhase::kIntentionallyDisabled ||
      motor_enable_phase == t800_safety::MotorEnablePhase::kEnabling;

  auto snapshot = health_->Update(state, motor_flags, 0, motor_flags_valid,
                                  &command, kResidentPeriod,
                                  !motor_not_ready);
  snapshot.profile = profile;
  if (profile_downgraded) {
    snapshot.status = t800_safety::MoreSevereStatus(
        snapshot.status, t800_safety::SafetyStatus::kLimited);
    snapshot.reason_mask |= t800_safety::kReasonProfileAuthorization;
  }
  if (motor_not_ready) {
    snapshot.status = t800_safety::MoreSevereStatus(
        snapshot.status, t800_safety::SafetyStatus::kLimited);
    snapshot.reason_mask |= t800_safety::kReasonMotorNotReady;
    if (motor_flags_valid) {
      for (std::size_t i = 0; i < t800_safety::kJointCount; ++i) {
        if (motor_flags.enable[i] == 0) snapshot.affected_joint_mask[i] = 1;
      }
    }
  } else if (motor_enable_phase == t800_safety::MotorEnablePhase::kFault) {
    snapshot.status = t800_safety::SafetyStatus::kFatal;
    snapshot.frame_fault = true;
    snapshot.reason_mask |= t800_safety::kReasonMotorFault |
                            t800_safety::kReasonFrameFault;
  }
  if (require_motor_debug && (!motor_debug_layout_valid || !motor_debug_fresh)) {
    snapshot.status = t800_safety::SafetyStatus::kFatal;
    snapshot.frame_fault = true;
    snapshot.reason_mask |= t800_safety::kReasonMotorFault |
                            t800_safety::kReasonDeadline |
                            t800_safety::kReasonFrameFault;
    LOG_EVERY_N(ERROR, 100)
        << "[T800SafetyRunner] MotorDebug missing, malformed, or older than 100 ms";
  }
  bool head_fault = false;
  for (std::size_t i = 0; i < t800_safety::kJointCount; ++i) {
    if (snapshot.head_fault_mask[i] == 0) continue;
    head_fault = true;
    command.q_des[i] = snapshot.sanitized_state.q[i];
    command.qd_des[i] = 0.0;
    command.kp[i] = 0.0;
    command.kd[i] = 0.0;
    command.tau_ff[i] = 0.0;
    command.full_torque[i] = 0.0;
  }
  if (snapshot.frame_fault || motor_not_ready) {
    command.q_des = snapshot.sanitized_state.q;
    command.qd_des.fill(0.0);
    command.kp.fill(0.0);
    command.kd.fill(0.0);
    command.tau_ff.fill(0.0);
    command.full_torque.fill(0.0);
    command.full_torque_enabled = false;
  }

  auto result = gate_->Evaluate(command, snapshot.sanitized_state,
                                kResidentPeriod, profile,
                                &snapshot.recovery_direction);
  const bool force_zero_actuation =
      snapshot.frame_fault || motor_not_ready || !result.accepted;
  if (force_zero_actuation) {
    for (std::size_t i = 0; i < t800_safety::kJointCount; ++i) {
      result.frame.q_des[i] = std::clamp(snapshot.sanitized_state.q[i],
                                         envelope_.joint(i).operational_position_lower,
                                         envelope_.joint(i).operational_position_upper);
    }
    result.frame.qd_des.fill(0.0);
    result.frame.kp.fill(0.0);
    result.frame.kd.fill(0.0);
    result.frame.tau_ff.fill(0.0);
    result.frame.full_torque.fill(0.0);
    result.frame.full_torque_enabled = false;
  }

  snapshot.reason_mask |= result.reason_mask;
  for (std::size_t i = 0; i < t800_safety::kJointCount; ++i) {
    if (result.affected_joint_mask[i] != 0) {
      snapshot.affected_joint_mask[i] = 1;
    }
    if (result.recovery_direction[i] !=
        t800_safety::RecoveryDirection::kNone) {
      snapshot.recovery_direction[i] = result.recovery_direction[i];
    }
  }
  snapshot.status = snapshot.frame_fault || !result.accepted
                        ? t800_safety::SafetyStatus::kFatal
                        : t800_safety::MoreSevereStatus(snapshot.status,
                                                        result.status);
  snapshot.frame_fault = snapshot.status == t800_safety::SafetyStatus::kFatal;
  snapshot.command_accepted = result.accepted && !snapshot.frame_fault;
  if (snapshot.frame_fault) {
    snapshot.command_decision = t800_safety::SafetyDecision::kFault;
  } else if (motor_not_ready) {
    snapshot.command_decision = t800_safety::SafetyDecision::kDegraded;
  } else if (snapshot.status == t800_safety::SafetyStatus::kRecovery) {
    snapshot.command_decision = t800_safety::SafetyDecision::kRecovery;
  } else if (profile_downgraded || head_fault) {
    snapshot.command_decision = t800_safety::SafetyDecision::kDegraded;
  } else {
    snapshot.command_decision = result.decision;
  }
  snapshot.published_at_steady_ns = now_ns;
  variant_store.Set("safety/t800_snapshot", snapshot);

  ++status_frames_since_report_;
  const bool status_changed =
      !have_reported_status_ || snapshot.status != last_reported_status_ ||
      snapshot.profile != last_reported_profile_ ||
      motor_enable_phase != last_reported_motor_enable_phase_ ||
      snapshot.reason_mask != last_reported_reason_mask_ ||
      motion_name != last_reported_motion_;
  const bool periodic_non_normal =
      snapshot.status != t800_safety::SafetyStatus::kNormal &&
      status_frames_since_report_ >= kPersistentStatusReportFrames;
  if (status_changed || periodic_non_normal) {
    std::ostringstream message;
    message << "[T800SafetyRunner] "
            << t800_safety::SafetyStatusMessage(snapshot.status)
            << ", motion=" << motion_name
            << ", profile=" << t800_safety::SafetyProfileName(snapshot.profile)
            << ", motor="
            << t800_safety::MotorEnablePhaseMessage(motor_enable_phase)
            << ", reasons="
            << t800_safety::SafetyReasonSummary(snapshot.reason_mask)
            << ", joints="
            << FormatAffectedJoints(snapshot, envelope_, state, result.frame);
    if (snapshot.status == t800_safety::SafetyStatus::kFatal) {
      LOG(ERROR) << message.str();
    } else if (snapshot.status == t800_safety::SafetyStatus::kNormal) {
      LOG(INFO) << message.str();
    } else {
      LOG(WARNING) << message.str();
    }
    have_reported_status_ = true;
    last_reported_status_ = snapshot.status;
    last_reported_profile_ = snapshot.profile;
    last_reported_motor_enable_phase_ = motor_enable_phase;
    last_reported_reason_mask_ = snapshot.reason_mask;
    last_reported_motion_ = motion_name;
    status_frames_since_report_ = 0;
  }

  Eigen::VectorXd safe_q(t800_safety::kJointCount);
  Eigen::VectorXd safe_qd(t800_safety::kJointCount);
  Eigen::VectorXd safe_kp(t800_safety::kJointCount);
  Eigen::VectorXd safe_kd(t800_safety::kJointCount);
  Eigen::VectorXd safe_tau_ff(t800_safety::kJointCount);
  for (std::size_t i = 0; i < t800_safety::kJointCount; ++i) {
    const Eigen::Index index = static_cast<Eigen::Index>(i);
    safe_q[index] = std::isfinite(result.frame.q_des[i]) ? result.frame.q_des[i] : 0.0;
    safe_qd[index] = std::isfinite(result.frame.qd_des[i]) ? result.frame.qd_des[i] : 0.0;
    safe_kp[index] = std::isfinite(result.frame.kp[i]) ? result.frame.kp[i] : 0.0;
    safe_kd[index] = std::isfinite(result.frame.kd[i]) ? result.frame.kd[i] : 0.0;
    safe_tau_ff[index] =
        std::isfinite(result.frame.tau_ff[i]) ? result.frame.tau_ff[i] : 0.0;
  }
  // SetCommandWithoutTorque intentionally leaves the full-torque field
  // untouched. Clear the complete command first so a rejected legacy torque
  // request cannot survive into sim_publish_runner or the hardware transform.
  data_store_->joint_info.SetZeroCommand();
  data_store_->joint_info.SetCommandWithoutTorque(safe_q, safe_qd, safe_tau_ff,
                                                   safe_kp, safe_kd);
}

bool T800MotorPreDriverSafetyRunner::Initialize() {
  const auto fail_closed = [this]() {
    if (data_store_) data_store_->motor_info.SetZeroCommand();
    auto& store = data::VariantStore::GetInstance();
    auto snapshot =
        store.GetCopy<t800_safety::T800SafetySnapshot>("safety/t800_snapshot")
            .value_or(t800_safety::T800SafetySnapshot{});
    snapshot.status = t800_safety::SafetyStatus::kFatal;
    snapshot.frame_fault = true;
    snapshot.command_accepted = false;
    snapshot.command_decision = t800_safety::SafetyDecision::kFault;
    snapshot.reason_mask |= t800_safety::kReasonFrameFault;
    store.Set("safety/t800_snapshot", snapshot);
    return false;
  };

  if (!data_store_ || !data_store_->model_param ||
      !t800_safety::IsT800Model(*data_store_->model_param)) {
    LOG(ERROR) << "[T800MotorPreDriverSafetyRunner] Invalid T800 model";
    return fail_closed();
  }

  const std::string contract_path = common::PathJoin(
      common::GlobalPathManager::GetInstance().GetConfigPath(),
      "safety/t800_safety_contract.json");
  std::string error;
  if (!t800_safety::T800JointEnvelope::Load(contract_path, &envelope_, &error)) {
    LOG(ERROR) << "[T800MotorPreDriverSafetyRunner] Failed to load " << contract_path
               << ": " << error;
    return fail_closed();
  }
  gate_ = std::make_unique<t800_safety::MotorCommandSafetyGate>(envelope_);
  initialized_ = true;
  return true;
}

bool T800MotorPreDriverSafetyRunner::Enter() {
  return initialized_ || Initialize();
}

void T800MotorPreDriverSafetyRunner::Run() {
  auto fail_closed = [this](std::uint32_t reason_mask) {
    if (data_store_) data_store_->motor_info.SetZeroCommand();
    auto& store = data::VariantStore::GetInstance();
    auto snapshot =
        store.GetCopy<t800_safety::T800SafetySnapshot>("safety/t800_snapshot")
            .value_or(t800_safety::T800SafetySnapshot{});
    snapshot.status = t800_safety::SafetyStatus::kFatal;
    snapshot.frame_fault = true;
    snapshot.command_accepted = false;
    snapshot.command_decision = t800_safety::SafetyDecision::kFault;
    snapshot.reason_mask |= reason_mask | t800_safety::kReasonFrameFault;
    // Preserve the joint gate's timestamp. A post-transform failure must not
    // make an old/missing joint safety snapshot appear fresh.
    store.Set("safety/t800_snapshot", snapshot);
  };

  if ((!initialized_ && !Initialize()) || !data_store_ || !gate_) {
    fail_closed(t800_safety::kReasonFrameFault);
    return;
  }

  const auto safety_snapshot =
      data::VariantStore::GetInstance().GetCopy<t800_safety::T800SafetySnapshot>(
          "safety/t800_snapshot");
  const std::uint64_t now_ns = t800_safety::SteadyNowNs();
  const bool snapshot_fresh =
      safety_snapshot && safety_snapshot->published_at_steady_ns != 0 &&
      now_ns >= safety_snapshot->published_at_steady_ns &&
      now_ns - safety_snapshot->published_at_steady_ns <=
          t800_safety::kSnapshotFreshnessLimitNs;
  if (!snapshot_fresh ||
      safety_snapshot->status == t800_safety::SafetyStatus::kFatal ||
      safety_snapshot->frame_fault ||
      !safety_snapshot->command_accepted) {
    fail_closed(!snapshot_fresh ? t800_safety::kReasonDeadline
                                : safety_snapshot->reason_mask);
    return;
  }

  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd tau;
  Eigen::VectorXd q_des;
  Eigen::VectorXd qd_des;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
  Eigen::VectorXd tau_ff;
  Eigen::VectorXd full_torque;
  data_store_->motor_info.GetState(data::JointInfoType::kPosition, q);
  data_store_->motor_info.GetState(data::JointInfoType::kVelocity, qd);
  data_store_->motor_info.GetState(data::JointInfoType::kTorque, tau);
  data_store_->motor_info.GetCommand(data::JointInfoType::kPosition, q_des);
  data_store_->motor_info.GetCommand(data::JointInfoType::kVelocity, qd_des);
  data_store_->motor_info.GetCommand(data::JointInfoType::kStiffness, kp);
  data_store_->motor_info.GetCommand(data::JointInfoType::kDamping, kd);
  data_store_->motor_info.GetCommand(data::JointInfoType::kFeedForwardTorque, tau_ff);
  data_store_->motor_info.GetCommand(data::JointInfoType::kTorque, full_torque);

  if (!HasExpectedSize(q) || !HasExpectedSize(qd) || !HasExpectedSize(tau) ||
      !HasExpectedSize(q_des) || !HasExpectedSize(qd_des) || !HasExpectedSize(kp) ||
      !HasExpectedSize(kd) || !HasExpectedSize(tau_ff) ||
      !HasExpectedSize(full_torque)) {
    LOG_EVERY_N(ERROR, 100)
        << "[T800MotorPreDriverSafetyRunner] Motor frame dimension mismatch";
    fail_closed(t800_safety::kReasonFrameFault);
    return;
  }

  t800_safety::T800JointState state;
  t800_safety::T800CommandFrame command;
  for (std::size_t i = 0; i < t800_safety::kJointCount; ++i) {
    const Eigen::Index index = static_cast<Eigen::Index>(i);
    state.q[i] = q[index];
    state.qd[i] = qd[index];
    state.tau[i] = tau[index];
    command.q_des[i] = q_des[index];
    command.qd_des[i] = qd_des[index];
    command.kp[i] = kp[index];
    command.kd[i] = kd[index];
    command.tau_ff[i] = tau_ff[index];
    command.full_torque[i] = full_torque[index];
  }
  command.full_torque_enabled = IsFullTorqueRequested(full_torque);

  auto post_snapshot = *safety_snapshot;
  bool post_transform_head_fault = false;
  for (const std::size_t head_index : {t800_safety::kHeadPitchIndex,
                                       t800_safety::kHeadYawIndex}) {
    const auto& head_joint = envelope_.joint(head_index);
    const bool state_finite = std::isfinite(state.q[head_index]) &&
                              std::isfinite(state.qd[head_index]) &&
                              std::isfinite(state.tau[head_index]);
    const bool command_finite = std::isfinite(command.q_des[head_index]) &&
                                std::isfinite(command.qd_des[head_index]) &&
                                std::isfinite(command.kp[head_index]) &&
                                std::isfinite(command.kd[head_index]) &&
                                std::isfinite(command.tau_ff[head_index]) &&
                                std::isfinite(command.full_torque[head_index]);
    const bool invalid_state_position =
        state_finite &&
        (state.q[head_index] <
             head_joint.hard_position_lower - kMotorCommandTolerance ||
         state.q[head_index] >
             head_joint.hard_position_upper + kMotorCommandTolerance);
    const bool invalid_state_velocity =
        state_finite &&
        std::abs(state.qd[head_index]) >
            head_joint.hard_velocity + kMotorCommandTolerance;
    const bool invalid_command_position =
        command_finite &&
        (command.q_des[head_index] <
             (safety_snapshot->profile == t800_safety::SafetyProfile::kPolicy
                  ? head_joint.operational_position_lower
                  : head_joint.hard_position_lower) -
                 kMotorCommandTolerance ||
         command.q_des[head_index] >
             (safety_snapshot->profile == t800_safety::SafetyProfile::kPolicy
                  ? head_joint.operational_position_upper
                  : head_joint.hard_position_upper) +
                 kMotorCommandTolerance);
    const bool invalid_command_velocity =
        command_finite &&
        std::abs(command.qd_des[head_index]) >
            head_joint.operational_velocity + kMotorCommandTolerance;
    const bool invalid_gain = command_finite &&
                              (command.kp[head_index] < 0.0 ||
                               command.kd[head_index] < 0.0);
    const bool invalid_full_torque =
        command_finite &&
        std::abs(command.full_torque[head_index]) > kMotorCommandTolerance;
    const bool invalid_feed_forward =
        command_finite &&
        std::abs(command.tau_ff[head_index]) >
            envelope_.joint(head_index).operational_effort + kMotorCommandTolerance;
    const double estimated_effort =
        state_finite && command_finite
            ? command.kp[head_index] *
                      (command.q_des[head_index] - state.q[head_index]) +
                  command.kd[head_index] *
                      (command.qd_des[head_index] - state.qd[head_index]) +
                  command.tau_ff[head_index]
            : 0.0;
    const bool invalid_pd_effort =
        state_finite && command_finite &&
        (!std::isfinite(estimated_effort) ||
         std::abs(estimated_effort) >
             envelope_.joint(head_index).operational_effort +
                 kMotorCommandTolerance);
    const bool isolate_head = post_snapshot.head_fault_mask[head_index] != 0 ||
                              !state_finite || !command_finite ||
                              invalid_state_position || invalid_state_velocity ||
                              invalid_command_position || invalid_command_velocity ||
                              invalid_gain || invalid_full_torque ||
                              invalid_feed_forward || invalid_pd_effort;
    if (!isolate_head) continue;

    post_transform_head_fault = true;
    post_snapshot.head_fault_mask[head_index] = 1;
    post_snapshot.affected_joint_mask[head_index] = 1;
    post_snapshot.health[head_index] = t800_safety::JointHealth::kFailed;
    post_snapshot.status = t800_safety::MoreSevereStatus(
        post_snapshot.status, t800_safety::SafetyStatus::kLimited);
    if (!state_finite || !command_finite) {
      post_snapshot.reason_mask |= t800_safety::kReasonNonFinite;
    }
    if (invalid_state_position || invalid_command_position) {
      post_snapshot.reason_mask |= t800_safety::kReasonHardPosition;
    }
    if (invalid_state_velocity || invalid_command_velocity) {
      post_snapshot.reason_mask |= t800_safety::kReasonVelocity;
    }
    if (invalid_gain) post_snapshot.reason_mask |= t800_safety::kReasonGain;
    if (invalid_full_torque) {
      post_snapshot.reason_mask |= t800_safety::kReasonFullTorqueUnsupported;
    }
    if (invalid_feed_forward) {
      post_snapshot.reason_mask |= t800_safety::kReasonEffort;
    }
    if (invalid_pd_effort) {
      post_snapshot.reason_mask |= t800_safety::kReasonPdEstimate;
    }
    const double safe_head_q =
        std::isfinite(post_snapshot.sanitized_state.q[head_index])
            ? post_snapshot.sanitized_state.q[head_index]
            : 0.0;
    state.q[head_index] = safe_head_q;
    state.qd[head_index] = 0.0;
    state.tau[head_index] = 0.0;
    command.q_des[head_index] = safe_head_q;
    command.qd_des[head_index] = 0.0;
    command.kp[head_index] = 0.0;
    command.kd[head_index] = 0.0;
    command.tau_ff[head_index] = 0.0;
    command.full_torque[head_index] = 0.0;
    full_torque[static_cast<Eigen::Index>(head_index)] = 0.0;
  }
  command.full_torque_enabled = IsFullTorqueRequested(full_torque);
  if (post_transform_head_fault) {
    post_snapshot.command_decision = t800_safety::SafetyDecision::kDegraded;
  }

  const auto result = gate_->Evaluate(command, state, safety_snapshot->profile,
                                      &safety_snapshot->recovery_direction);
  if (!result.accepted) {
    LOG_EVERY_N(ERROR, 100)
        << "[T800MotorPreDriverSafetyRunner] "
        << t800_safety::SafetyStatusMessage(t800_safety::SafetyStatus::kFatal)
        << ", reasons="
        << t800_safety::SafetyReasonSummary(result.reason_mask);
    fail_closed(result.reason_mask);
    return;
  }

  post_snapshot.status = t800_safety::MoreSevereStatus(post_snapshot.status,
                                                        result.status);
  for (std::size_t i = 0; i < t800_safety::kJointCount; ++i) {
    if (result.affected_joint_mask[i] != 0) {
      post_snapshot.affected_joint_mask[i] = 1;
    }
    if (result.recovery_direction[i] !=
        t800_safety::RecoveryDirection::kNone) {
      post_snapshot.recovery_direction[i] = result.recovery_direction[i];
    }
  }
  if (post_snapshot.status == t800_safety::SafetyStatus::kRecovery) {
    post_snapshot.command_decision = t800_safety::SafetyDecision::kRecovery;
  } else if (post_snapshot.status == t800_safety::SafetyStatus::kLimited) {
    post_snapshot.command_decision = post_transform_head_fault
                                         ? t800_safety::SafetyDecision::kDegraded
                                         : t800_safety::SafetyDecision::kClipped;
  }
  if (post_transform_head_fault ||
      result.status != t800_safety::SafetyStatus::kNormal) {
    data::VariantStore::GetInstance().Set("safety/t800_snapshot", post_snapshot);
    LOG_EVERY_N(WARNING, 500)
        << "[T800MotorPreDriverSafetyRunner] "
        << t800_safety::SafetyStatusMessage(post_snapshot.status)
        << ", stage=post_transform, profile="
        << t800_safety::SafetyProfileName(post_snapshot.profile)
        << ", reasons="
        << t800_safety::SafetyReasonSummary(post_snapshot.reason_mask);
  }

  Eigen::VectorXd safe_q(t800_safety::kJointCount);
  Eigen::VectorXd safe_qd(t800_safety::kJointCount);
  Eigen::VectorXd safe_kp(t800_safety::kJointCount);
  Eigen::VectorXd safe_kd(t800_safety::kJointCount);
  Eigen::VectorXd safe_tau_ff(t800_safety::kJointCount);
  for (std::size_t i = 0; i < t800_safety::kJointCount; ++i) {
    const Eigen::Index index = static_cast<Eigen::Index>(i);
    safe_q[index] = result.frame.q_des[i];
    safe_qd[index] = result.frame.qd_des[i];
    safe_kp[index] = result.frame.kp[i];
    safe_kd[index] = result.frame.kd[i];
    safe_tau_ff[index] = result.frame.tau_ff[i];
  }
  // The pre-driver frame must also erase any torque-channel value left by the
  // transform before MotorRunner consumes motor_info.
  data_store_->motor_info.SetZeroCommand();
  data_store_->motor_info.SetCommandWithoutTorque(safe_q, safe_qd, safe_tau_ff,
                                                   safe_kp, safe_kd);
}

}  // namespace runner

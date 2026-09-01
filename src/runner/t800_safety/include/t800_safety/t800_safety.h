#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include <Eigen/Dense>

namespace data {
class ModelParam;
}

namespace runner::t800_safety {

constexpr std::size_t kJointCount = 25;
constexpr std::size_t kHeadPitchIndex = 23;
constexpr std::size_t kHeadYawIndex = 24;
constexpr int kHeadRecoveryHealthyFrames = 50;
constexpr int kHeadTrackingFaultFrames = 50;
constexpr int kHeadVelocityMismatchFaultFrames = 10;

struct JointEnvelope {
  std::string sdk_name;
  int sdk_index = -1;
  double hard_position_lower = 0.0;
  double hard_position_upper = 0.0;
  double hard_velocity = 0.0;
  double hard_effort = 0.0;
  double operational_position_lower = 0.0;
  double operational_position_upper = 0.0;
  double operational_velocity = 0.0;
  double operational_effort = 0.0;
};

class T800JointEnvelope {
 public:
  static bool Load(const std::string& path, T800JointEnvelope* out, std::string* error = nullptr);
  bool Validate(std::string* error = nullptr) const;

  int schema_version() const { return schema_version_; }
  const std::string& contract_version() const { return contract_version_; }
  const std::string& qualification() const { return qualification_; }
  const std::string& source_urdf_sha256() const { return source_urdf_sha256_; }
  const JointEnvelope& joint(std::size_t index) const { return joints_[index]; }

 private:
  int schema_version_ = 0;
  std::string contract_version_;
  std::string robot_;
  std::string qualification_;
  std::string source_urdf_sha256_;
  std::array<JointEnvelope, kJointCount> joints_{};
};

struct T800CommandFrame {
  std::array<double, kJointCount> q_des{};
  std::array<double, kJointCount> qd_des{};
  std::array<double, kJointCount> kp{};
  std::array<double, kJointCount> kd{};
  std::array<double, kJointCount> tau_ff{};
  std::array<double, kJointCount> full_torque{};
  bool full_torque_enabled = false;
  std::uint64_t generation = 0;
  std::uint64_t motion_epoch = 0;
  std::uint64_t producer_sequence = 0;
};

struct T800JointState {
  std::array<double, kJointCount> q{};
  std::array<double, kJointCount> qd{};
  std::array<double, kJointCount> tau{};
};

struct T800MotorFlags {
  std::array<std::uint8_t, kJointCount> offline{};
  std::array<std::uint8_t, kJointCount> enable{};
  std::array<int, kJointCount> error_code{};
};

enum class JointHealth : std::uint8_t { kHealthy, kSuspect, kFailed, kRecovering };

enum class SafetyDecision : std::uint8_t { kAccepted, kClipped, kDegraded, kFault };

enum SafetyReason : std::uint32_t {
  kReasonNone = 0,
  kReasonNonFinite = 1u << 0,
  kReasonHardPosition = 1u << 1,
  kReasonVelocity = 1u << 2,
  kReasonSlew = 1u << 3,
  kReasonEffort = 1u << 4,
  kReasonPdEstimate = 1u << 5,
  kReasonFullTorqueUnsupported = 1u << 6,
  kReasonMotorFault = 1u << 7,
  kReasonFrameFault = 1u << 8,
  kReasonGain = 1u << 9,
  kReasonDeadline = 1u << 10,
  kReasonFeedbackJump = 1u << 11,
  kReasonFeedbackTracking = 1u << 12,
  kReasonFeedbackMismatch = 1u << 13,
};

struct T800SafetySnapshot {
  T800JointState sanitized_state{};
  std::array<JointHealth, kJointCount> health{};
  std::array<std::uint8_t, kJointCount> head_fault_mask{};
  bool frame_fault = false;
  bool command_accepted = false;
  SafetyDecision command_decision = SafetyDecision::kFault;
  std::uint32_t reason_mask = kReasonNone;
  std::uint64_t generation = 0;
  std::uint64_t motion_epoch = 0;
  std::uint64_t published_at_steady_ns = 0;
};

class HeadJointHealthMonitor {
 public:
  explicit HeadJointHealthMonitor(const T800JointEnvelope& envelope);
  T800SafetySnapshot Update(const T800JointState& raw, const T800MotorFlags& flags,
                            std::uint64_t motion_epoch = 0, bool motor_flags_valid = false,
                            const T800CommandFrame* requested = nullptr,
                            double sample_period = 0.002);
  void Reset();

 private:
  const T800JointEnvelope& envelope_;
  T800SafetySnapshot snapshot_{};
  std::array<bool, kJointCount> have_last_good_{};
  std::array<double, kJointCount> last_good_q_{};
  std::array<bool, kJointCount> have_previous_sample_{};
  std::array<double, kJointCount> previous_sample_q_{};
  std::array<int, kJointCount> healthy_frames_{};
  std::array<int, kJointCount> tracking_fault_frames_{};
  std::array<int, kJointCount> velocity_mismatch_frames_{};
};

struct GateResult {
  SafetyDecision decision = SafetyDecision::kFault;
  std::uint32_t reason_mask = kReasonNone;
  bool accepted = false;
  T800CommandFrame frame{};
};

class JointCommandSafetyGate {
 public:
  explicit JointCommandSafetyGate(const T800JointEnvelope& envelope);
  GateResult Evaluate(const T800CommandFrame& requested, const T800JointState& measured,
                     double producer_dt);
  void Reset();
  const T800CommandFrame& last_safe() const { return last_safe_; }

 private:
  const T800JointEnvelope& envelope_;
  T800CommandFrame last_safe_{};
  bool have_last_safe_ = false;
};

// Final post-transform gate. Position geometry for parallel actuators is not
// inferred from joint URDF limits here; this gate instead guarantees that the
// transformed motor frame is finite, uses non-negative gains, never uses the
// unsupported full-torque channel, and keeps direct-drive position, velocity,
// and effort inside the provisional joint envelope before motor_runner consumes
// it. The parallel ankles are deliberately excluded from joint-derived limits
// until actuator-space limits and the qualified transform/Jacobian are
// available.
class MotorCommandSafetyGate {
 public:
  explicit MotorCommandSafetyGate(const T800JointEnvelope& envelope) : envelope_(envelope) {}
  GateResult Evaluate(const T800CommandFrame& requested,
                      const T800JointState& measured) const;

 private:
  const T800JointEnvelope& envelope_;
};

bool GetT800SanitizedState(Eigen::VectorXd* q, Eigen::VectorXd* qd,
                           T800SafetySnapshot* snapshot = nullptr);
void MaskFailedHeadActions(const T800SafetySnapshot& snapshot,
                           const Eigen::VectorXi& action_to_joint,
                           Eigen::VectorXd* actions);
bool IsT800Model(const data::ModelParam& model);

}  // namespace runner::t800_safety

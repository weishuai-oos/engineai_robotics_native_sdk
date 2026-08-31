#pragma once

#include <Eigen/Dense>

#include "joint_info/joint_info.h"

namespace runner::motion_transition {

struct JointCommand {
  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
  Eigen::VectorXd tau_ff;

  void Resize(int joint_count);
  bool IsValid(int joint_count) const;
};

struct EntryTransitionConfig {
  bool enabled = true;
  double nominal_duration = 0.16;
  double min_duration = 0.10;
  double max_duration = 0.28;
  double max_joint_velocity = 8.0;
  double max_joint_acceleration = 120.0;
  double reference_pose_weight = 0.35;
  double source_command_tracking_error = 0.75;
};

// Captures the command currently owned by the outgoing motion. The snapshot is
// intentionally fixed for the whole transition; measured state remains the
// policy feedback and is only used here to reject a stale source command.
bool CaptureJointCommand(data::JointInfo& joint_info, int joint_count, JointCommand* command);

class EntryCommandTransition {
 public:
  bool Configure(const EntryTransitionConfig& config);

  // fallback_target supplies safe dimensions and gains if the outgoing command
  // is unavailable. actual_q/actual_qd are sampled at the switch instant.
  bool Start(const JointCommand& source, const Eigen::VectorXd& actual_q,
             const Eigen::VectorXd& actual_qd, const JointCommand& fallback_target);

  // The destination policy must be evaluated before Apply() on every cycle.
  // reference_q is only a decaying pose hint; the final command always equals
  // the live policy target.
  bool Apply(const JointCommand& live_target, const Eigen::VectorXd& reference_q,
             const Eigen::VectorXd& actual_q, double control_dt, JointCommand* output);

  void Reset();
  bool IsActive() const { return active_; }
  bool IsComplete() const { return started_ && !active_; }
  double duration() const { return duration_; }
  double elapsed() const { return elapsed_; }

 private:
  bool InitializeDuration(const JointCommand& live_target, const Eigen::VectorXd& reference_q);

  EntryTransitionConfig config_;
  JointCommand source_;
  bool configured_ = false;
  bool started_ = false;
  bool active_ = false;
  bool duration_initialized_ = false;
  bool tracking_warning_emitted_ = false;
  double duration_ = 0.0;
  double elapsed_ = 0.0;
};

}  // namespace runner::motion_transition

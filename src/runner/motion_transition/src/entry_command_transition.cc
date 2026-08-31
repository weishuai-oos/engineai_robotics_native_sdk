#include "motion_transition/entry_command_transition.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <glog/logging.h>

namespace runner::motion_transition {
namespace {

constexpr double kQuinticMaxFirstDerivative = 1.875;
constexpr double kQuinticMaxSecondDerivative = 5.773502691896258;

bool IsFiniteNonNegative(const Eigen::VectorXd& values) {
  return values.allFinite() && (values.array() >= 0.0).all();
}

Eigen::VectorXd Lerp(const Eigen::VectorXd& start, const Eigen::VectorXd& end, double alpha) {
  return start + alpha * (end - start);
}

double QuinticSmoothStep(double phase) {
  const double s = std::clamp(phase, 0.0, 1.0);
  return s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
}

double QuinticSmoothStepDerivative(double phase, double duration) {
  if (duration <= 0.0) return 0.0;
  const double s = std::clamp(phase, 0.0, 1.0);
  return 30.0 * s * s * (1.0 - s) * (1.0 - s) / duration;
}

double MaxAbs(const Eigen::VectorXd& values) {
  return values.size() == 0 ? 0.0 : values.cwiseAbs().maxCoeff();
}

}  // namespace

void JointCommand::Resize(int joint_count) {
  q.setZero(joint_count);
  qd.setZero(joint_count);
  kp.setZero(joint_count);
  kd.setZero(joint_count);
  tau_ff.setZero(joint_count);
}

bool JointCommand::IsValid(int joint_count) const {
  return q.size() == joint_count && qd.size() == joint_count && kp.size() == joint_count &&
         kd.size() == joint_count && tau_ff.size() == joint_count && q.allFinite() && qd.allFinite() &&
         IsFiniteNonNegative(kp) && IsFiniteNonNegative(kd) && tau_ff.allFinite();
}

bool CaptureJointCommand(data::JointInfo& joint_info, int joint_count, JointCommand* command) {
  if (!command || joint_count <= 0) return false;
  command->Resize(joint_count);
  joint_info.GetCommand(data::JointInfoType::kPosition, command->q);
  joint_info.GetCommand(data::JointInfoType::kVelocity, command->qd);
  joint_info.GetCommand(data::JointInfoType::kStiffness, command->kp);
  joint_info.GetCommand(data::JointInfoType::kDamping, command->kd);
  joint_info.GetCommand(data::JointInfoType::kFeedForwardTorque, command->tau_ff);
  return command->IsValid(joint_count);
}

bool EntryCommandTransition::Configure(const EntryTransitionConfig& config) {
  if (!std::isfinite(config.nominal_duration) || !std::isfinite(config.min_duration) ||
      !std::isfinite(config.max_duration) || !std::isfinite(config.max_joint_velocity) ||
      !std::isfinite(config.max_joint_acceleration) || !std::isfinite(config.reference_pose_weight) ||
      !std::isfinite(config.source_command_tracking_error) || config.nominal_duration < 0.0 ||
      config.min_duration < 0.0 || config.max_duration < config.min_duration ||
      config.nominal_duration < config.min_duration || config.nominal_duration > config.max_duration ||
      config.max_joint_velocity <= 0.0 || config.max_joint_acceleration <= 0.0 ||
      config.reference_pose_weight < 0.0 || config.reference_pose_weight > 1.0 ||
      config.source_command_tracking_error < 0.0) {
    LOG(ERROR) << "Invalid entry transition configuration";
    configured_ = false;
    return false;
  }
  config_ = config;
  configured_ = true;
  Reset();
  return true;
}

bool EntryCommandTransition::Start(const JointCommand& source, const Eigen::VectorXd& actual_q,
                                   const Eigen::VectorXd& actual_qd, const JointCommand& fallback_target) {
  if (!configured_ || actual_q.size() <= 0 || actual_qd.size() != actual_q.size() ||
      !actual_q.allFinite() || !actual_qd.allFinite() || !fallback_target.IsValid(actual_q.size())) {
    LOG(ERROR) << "Cannot start entry transition with invalid command/state dimensions";
    return false;
  }

  const int joint_count = actual_q.size();
  source_ = source.IsValid(joint_count) ? source : fallback_target;
  if (!source.IsValid(joint_count)) {
    source_.q = actual_q;
    source_.qd = actual_qd;
    LOG(WARNING) << "Outgoing joint command unavailable; entry transition starts from measured state";
  }

  if (config_.source_command_tracking_error > 0.0) {
    const Eigen::VectorXd source_error = source_.q - actual_q;
    const double max_error = MaxAbs(source_error);
    if (max_error > config_.source_command_tracking_error) {
      source_.q = actual_q + source_error.cwiseMax(-config_.source_command_tracking_error)
                                   .cwiseMin(config_.source_command_tracking_error);
      LOG(WARNING) << "Outgoing position command differs from measured state by " << max_error
                   << " rad; clamped transition source to +/-"
                   << config_.source_command_tracking_error << " rad around measured q";
    }
  }

  started_ = true;
  active_ = config_.enabled && config_.max_duration > 0.0;
  duration_initialized_ = false;
  tracking_warning_emitted_ = false;
  duration_ = active_ ? config_.nominal_duration : 0.0;
  elapsed_ = 0.0;
  return true;
}

bool EntryCommandTransition::InitializeDuration(const JointCommand& live_target,
                                                const Eigen::VectorXd& reference_q) {
  if (!live_target.IsValid(source_.q.size())) return false;

  double max_position_delta = MaxAbs(live_target.q - source_.q);
  if (reference_q.size() == source_.q.size() && reference_q.allFinite() &&
      config_.reference_pose_weight > 0.0) {
    max_position_delta = std::max(max_position_delta, MaxAbs(reference_q - source_.q));
  }

  const double velocity_duration =
      kQuinticMaxFirstDerivative * max_position_delta / config_.max_joint_velocity;
  const double acceleration_duration =
      std::sqrt(kQuinticMaxSecondDerivative * max_position_delta / config_.max_joint_acceleration);
  const double requested_duration =
      std::max({config_.nominal_duration, config_.min_duration, velocity_duration, acceleration_duration});
  duration_ = std::clamp(requested_duration, config_.min_duration, config_.max_duration);
  duration_initialized_ = true;

  if (requested_duration > config_.max_duration + std::numeric_limits<double>::epsilon()) {
    LOG(WARNING) << "Entry transition required " << requested_duration
                 << "s to satisfy configured q/qd limits, capped at " << config_.max_duration
                 << "s; consider increasing max_duration for real-robot tests";
  }
  LOG(INFO) << "Entry command transition started: duration=" << duration_
            << "s, max_initial_q_delta=" << max_position_delta << " rad";
  return true;
}

bool EntryCommandTransition::Apply(const JointCommand& live_target, const Eigen::VectorXd& reference_q,
                                   const Eigen::VectorXd& actual_q, double control_dt, JointCommand* output) {
  if (!output || !started_ || !live_target.IsValid(source_.q.size()) ||
      actual_q.size() != source_.q.size() || !actual_q.allFinite() || !std::isfinite(control_dt) ||
      control_dt <= 0.0) {
    LOG(ERROR) << "Cannot apply entry transition with invalid target/state/control_dt";
    return false;
  }

  if (!active_) {
    *output = live_target;
    return true;
  }
  if (!duration_initialized_ && !InitializeDuration(live_target, reference_q)) return false;
  if (duration_ <= 0.0) {
    *output = live_target;
    active_ = false;
    return true;
  }

  elapsed_ = std::min(elapsed_ + control_dt, duration_);
  if (duration_ - elapsed_ <= 1e-12 * std::max(1.0, duration_)) {
    elapsed_ = duration_;
  }
  const double normalized_time = elapsed_ / duration_;
  const double alpha = QuinticSmoothStep(normalized_time);
  const double alpha_dot = QuinticSmoothStepDerivative(normalized_time, duration_);

  Eigen::VectorXd guided_q = live_target.q;
  if (reference_q.size() == source_.q.size() && reference_q.allFinite()) {
    const double hint_weight = config_.reference_pose_weight * (1.0 - alpha);
    guided_q = Lerp(live_target.q, reference_q, hint_weight);
  }

  output->q = Lerp(source_.q, guided_q, alpha);
  Eigen::VectorXd path_derivative = guided_q - source_.q;
  if (reference_q.size() == source_.q.size() && reference_q.allFinite()) {
    path_derivative -= alpha * config_.reference_pose_weight * (reference_q - live_target.q);
  }
  output->qd = Lerp(source_.qd, live_target.qd, alpha) + alpha_dot * path_derivative;
  output->qd = output->qd.cwiseMax(-config_.max_joint_velocity).cwiseMin(config_.max_joint_velocity);
  output->kp = Lerp(source_.kp, live_target.kp, alpha);
  output->kd = Lerp(source_.kd, live_target.kd, alpha);
  output->tau_ff = Lerp(source_.tau_ff, live_target.tau_ff, alpha);

  const double tracking_error = MaxAbs(output->q - actual_q);
  if (!tracking_warning_emitted_ && config_.source_command_tracking_error > 0.0 &&
      tracking_error > config_.source_command_tracking_error) {
    LOG(WARNING) << "Entry transition command tracking error is " << tracking_error
                 << " rad; verify balance/contact before shortening the transition";
    tracking_warning_emitted_ = true;
  }

  if (elapsed_ >= duration_) {
    // Quintic alpha is exactly one here. Assign the live target explicitly so
    // reference guidance and floating-point roundoff cannot leak past the bridge.
    *output = live_target;
    active_ = false;
    LOG(INFO) << "Entry command transition completed";
  }
  return true;
}

void EntryCommandTransition::Reset() {
  source_ = JointCommand{};
  started_ = false;
  active_ = false;
  duration_initialized_ = false;
  tracking_warning_emitted_ = false;
  duration_ = 0.0;
  elapsed_ = 0.0;
}

}  // namespace runner::motion_transition

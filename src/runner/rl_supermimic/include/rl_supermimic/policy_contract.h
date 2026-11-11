#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "motion_transition/heading_alignment.h"

namespace runner::supermimic {

constexpr int kActions = 25;
constexpr int kObservation = 87;
constexpr int kHistory = 10;
constexpr int kGoal = 650;
constexpr int kInput = kObservation * kHistory + kGoal;
using Observation = Eigen::Matrix<float, kObservation, 1>;
using History = Eigen::Matrix<float, kHistory, kObservation, Eigen::RowMajor>;
using Input = Eigen::Matrix<float, kInput, 1>;

using motion_transition::AlignYaw;

inline Eigen::Matrix3d AlignReferenceYaw(const Eigen::Matrix3d& actual_initial,
                                        const Eigen::Matrix3d& reference_initial,
                                        const Eigen::Matrix3d& reference) {
  return AlignYaw(actual_initial, reference_initial) * reference;
}

inline Observation BuildObservation(const Eigen::VectorXd& q, const Eigen::VectorXd& qd,
                                    const Eigen::Matrix3d& root_rotation,
                                    const Eigen::Vector3d& angular_velocity_body,
                                    const Eigen::Matrix3d& reference_rotation,
                                    const Eigen::VectorXd& default_q,
                                    const Eigen::VectorXd& previous_action) {
  if (q.size() != kActions || qd.size() != kActions || default_q.size() != kActions ||
      previous_action.size() != kActions) throw std::invalid_argument("SuperMimic observation joint dimension");
  Observation result;
  result.segment<25>(0) = (q - default_q).cast<float>();
  result.segment<25>(25) = qd.cast<float>();
  result.segment<3>(50) = angular_velocity_body.cast<float>();
  result.segment<3>(53) = (-root_rotation.row(2).transpose()).cast<float>();
  result.segment<25>(56) = previous_action.cast<float>();
  const Eigen::Matrix3d relative = root_rotation.transpose() * reference_rotation;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 2; ++col) result(81 + 2 * row + col) = relative(row, col);
  if (!result.allFinite()) throw std::invalid_argument("Nonfinite SuperMimic observation");
  return result;
}

inline void PushObservation(const Observation& observation, bool first, History* history) {
  if (!history || !observation.allFinite()) throw std::invalid_argument("Invalid SuperMimic history");
  if (first) history->rowwise() = observation.transpose();
  else {
    history->topRows<kHistory - 1>() = history->bottomRows<kHistory - 1>().eval();
    history->row(kHistory - 1) = observation.transpose();
  }
}

inline Input BuildInput(const History& history, const Eigen::VectorXf& goal) {
  if (goal.size() != kGoal || !goal.allFinite() || !history.allFinite())
    throw std::invalid_argument("Invalid SuperMimic policy input");
  Input result;
  result.head<kObservation * kHistory>() = Eigen::Map<const Eigen::VectorXf>(history.data(), history.size());
  result.tail<kGoal>() = goal;
  return result;
}

inline Eigen::VectorXd ActionTarget(const Eigen::VectorXd& default_q, const Eigen::VectorXd& scale,
                                     const Eigen::VectorXd& action) {
  if (default_q.size() != kActions || scale.size() != kActions || action.size() != kActions ||
      !default_q.allFinite() || !scale.allFinite() || !action.allFinite())
    throw std::invalid_argument("Invalid SuperMimic action target");
  // Use the model output unchanged: raw-action exports do not embed a clamp.
  return default_q + scale.cwiseProduct(action);
}

class Sequence {
 public:
  bool Configure(int frames, double hold_seconds, double control_dt, int skip_terminal_hold_frames = 0) {
    if (frames < 2 || !std::isfinite(hold_seconds) || hold_seconds < 0.0 ||
        !std::isfinite(control_dt) || control_dt <= 0.0 || hold_seconds / control_dt > 1000000.0 ||
        skip_terminal_hold_frames < 0 || skip_terminal_hold_frames > frames - 2) return false;
    // Shorten playback only; leave the exported reference and look-ahead goals intact.
    // Keep the same N-1 control intervals as a standalone dynamic reference clip.
    frames_ = frames - skip_terminal_hold_frames;
    total_steps_ = frames_ - 1 + static_cast<int>(std::llround(hold_seconds / control_dt));
    step_ = 0;
    return true;
  }
  int frame() const { return std::min(step_, frames_ - 2); }
  void Advance() { step_ = std::min(step_ + 1, total_steps_); }
  bool finished() const { return step_ >= total_steps_; }
  void Reset() { step_ = 0; }
 private:
  int frames_ = 2;
  int total_steps_ = 1;
  int step_ = 0;
};

}  // namespace runner::supermimic

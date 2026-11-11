#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace runner::motion_transition {

// Fit one fixed world-Z rotation from reference to actual using the full body
// orientation. Extracting two separate Euler yaw angles becomes ill-conditioned
// when a body's forward axis is nearly vertical, as in prone/supine recovery.
inline Eigen::Matrix3d AlignYaw(const Eigen::Matrix3d& actual_initial,
                                const Eigen::Matrix3d& reference_initial) {
  if (!actual_initial.allFinite() || !reference_initial.allFinite()) {
    throw std::invalid_argument("Nonfinite heading alignment orientation");
  }

  const Eigen::Matrix3d relative = reference_initial * actual_initial.transpose();
  double c = relative(0, 0) + relative(1, 1);
  double s = relative(0, 1) - relative(1, 0);
  if (std::hypot(c, s) <= 1e-10) {
    // A 180-degree relative tilt has no unique least-squares yaw. Keep the
    // result finite and deterministic by matching the body axis with the
    // strongest shared horizontal projection. The score is yaw-invariant;
    // resolving numerical ties by axis order preserves that invariance too.
    double best_score = -1.0;
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Vector2d actual = actual_initial.col(axis).head<2>();
      const Eigen::Vector2d reference = reference_initial.col(axis).head<2>();
      const double score = actual.squaredNorm() * reference.squaredNorm();
      if (score > best_score + 1e-12) {
        best_score = score;
        c = reference.dot(actual);
        s = reference.x() * actual.y() - reference.y() * actual.x();
      }
    }
    if (best_score <= 1e-20) {
      throw std::invalid_argument("Invalid heading alignment orientation");
    }
  }
  return Eigen::AngleAxisd(std::atan2(s, c), Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

}  // namespace runner::motion_transition

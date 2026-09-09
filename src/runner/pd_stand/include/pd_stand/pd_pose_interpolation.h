#pragma once

#include <algorithm>

#include <Eigen/Dense>

namespace runner::pd_stand {

// Quintic interpolation with zero velocity and acceleration at both endpoints.
inline void InterpolateQuintic(const Eigen::VectorXd& start, const Eigen::VectorXd& target, double duration,
                               double time, Eigen::VectorXd& position, Eigen::VectorXd& velocity) {
  const double normalized_time = std::clamp(time / duration, 0.0, 1.0);
  const double t2 = normalized_time * normalized_time;
  const double t3 = t2 * normalized_time;
  const double t4 = t3 * normalized_time;
  const double t5 = t4 * normalized_time;
  const double position_scale = 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
  const double velocity_scale = (30.0 * t2 - 60.0 * t3 + 30.0 * t4) / duration;
  const Eigen::VectorXd delta = target - start;
  position = start + position_scale * delta;
  velocity = velocity_scale * delta;
}

}  // namespace runner::pd_stand

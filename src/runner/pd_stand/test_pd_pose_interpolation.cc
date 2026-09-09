#include "pd_stand/pd_pose_interpolation.h"

#include <cmath>

#include <gtest/gtest.h>

namespace runner::pd_stand {
namespace {

TEST(PdPoseInterpolationTest, MatchesEndpointsAndKnownMidpoint) {
  const Eigen::VectorXd start = Eigen::Vector2d(-1.0, 2.0);
  const Eigen::VectorXd target = Eigen::Vector2d(3.0, -2.0);
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;

  InterpolateQuintic(start, target, 2.0, 0.0, position, velocity);
  EXPECT_TRUE(position.isApprox(start));
  EXPECT_TRUE(velocity.isZero(1e-12));

  InterpolateQuintic(start, target, 2.0, 1.0, position, velocity);
  EXPECT_TRUE(position.isApprox(Eigen::Vector2d(1.0, 0.0), 1e-12));
  EXPECT_TRUE(velocity.isApprox(Eigen::Vector2d(3.75, -3.75), 1e-12));

  InterpolateQuintic(start, target, 2.0, 2.0, position, velocity);
  EXPECT_TRUE(position.isApprox(target));
  EXPECT_TRUE(velocity.isZero(1e-12));
}

TEST(PdPoseInterpolationTest, NumericalVelocityMatchesAnalyticVelocity) {
  const Eigen::VectorXd start = Eigen::Vector3d(-0.4, 0.2, 1.1);
  const Eigen::VectorXd target = Eigen::Vector3d(0.8, -0.5, -0.3);
  constexpr double duration = 1.6;
  constexpr double time = 0.72;
  constexpr double epsilon = 1e-6;
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd before;
  Eigen::VectorXd after;

  InterpolateQuintic(start, target, duration, time, position, velocity);
  InterpolateQuintic(start, target, duration, time - epsilon, before, velocity);
  InterpolateQuintic(start, target, duration, time + epsilon, after, velocity);
  const Eigen::VectorXd numerical_velocity = (after - before) / (2.0 * epsilon);

  InterpolateQuintic(start, target, duration, time, position, velocity);
  EXPECT_TRUE(numerical_velocity.isApprox(velocity, 1e-8));
}

TEST(PdPoseInterpolationTest, ClampsOutsideDurationWithoutOvershoot) {
  const Eigen::VectorXd start = Eigen::Vector2d(2.0, -3.0);
  const Eigen::VectorXd target = Eigen::Vector2d(-4.0, 5.0);
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;

  InterpolateQuintic(start, target, 1.0, -2.0, position, velocity);
  EXPECT_TRUE(position.isApprox(start));
  EXPECT_TRUE(velocity.isZero(1e-12));

  InterpolateQuintic(start, target, 1.0, 3.0, position, velocity);
  EXPECT_TRUE(position.isApprox(target));
  EXPECT_TRUE(velocity.isZero(1e-12));

  for (double time = 0.0; time <= 1.0; time += 0.01) {
    InterpolateQuintic(start, target, 1.0, time, position, velocity);
    for (int i = 0; i < position.size(); ++i) {
      EXPECT_GE(position[i], std::min(start[i], target[i]));
      EXPECT_LE(position[i], std::max(start[i], target[i]));
    }
  }
}

}  // namespace
}  // namespace runner::pd_stand

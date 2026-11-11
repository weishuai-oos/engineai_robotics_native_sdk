#include "motion_transition/heading_alignment.h"

#include <gtest/gtest.h>
#include <array>
#include <limits>

namespace runner::motion_transition {
namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Matrix3d Rotate(double angle, const Eigen::Vector3d& axis) {
  return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

Eigen::Matrix3d Z(double angle) { return Rotate(angle, Eigen::Vector3d::UnitZ()); }
Eigen::Matrix3d Y(double angle) { return Rotate(angle, Eigen::Vector3d::UnitY()); }
Eigen::Matrix3d X(double angle) { return Rotate(angle, Eigen::Vector3d::UnitX()); }

void ExpectPureYaw(const Eigen::Matrix3d& alignment) {
  EXPECT_TRUE(alignment.allFinite());
  EXPECT_TRUE((alignment.transpose() * alignment).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_NEAR(alignment.determinant(), 1.0, 1e-12);
  EXPECT_TRUE(alignment.col(2).isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
}

TEST(HeadingAlignmentTest, PreservesUprightAndOppositeHeadings) {
  for (double heading : {0.0, 0.4, -2.8, kPi, -kPi}) {
    const Eigen::Matrix3d reference = Z(0.7);
    const Eigen::Matrix3d alignment = AlignYaw(Z(heading) * reference, reference);
    ExpectPureYaw(alignment);
    EXPECT_TRUE(alignment.isApprox(Z(heading), 1e-12));
  }
}

TEST(HeadingAlignmentTest, MatchesYawForSameTiltAtProneAndSupineSingularity) {
  for (double pitch : {-kPi / 2.0, -kPi / 2.0 + 1e-10, 0.0,
                       kPi / 2.0 - 1e-10, kPi / 2.0}) {
    const Eigen::Matrix3d reference = Z(-0.9) * Y(pitch) * X(0.2);
    const Eigen::Matrix3d alignment = AlignYaw(Z(2.4) * reference, reference);
    ExpectPureYaw(alignment);
    EXPECT_TRUE(alignment.isApprox(Z(2.4), 1e-12));
  }
}

TEST(HeadingAlignmentTest, SmallWorldTiltDoesNotCreateSpuriousYawNearProne) {
  const Eigen::Matrix3d reference = Z(-0.9) * Y(kPi / 2.0 - 0.02);
  const double heading = 2.3;
  for (double roll : {-0.03, -0.01, 0.0, 0.01, 0.03}) {
    const Eigen::Matrix3d actual = Z(heading) * X(roll) * reference;
    const Eigen::Matrix3d alignment = AlignYaw(actual, reference);
    EXPECT_TRUE(alignment.isApprox(Z(heading), 1e-12));
  }
}

TEST(HeadingAlignmentTest, RecoversProneCaseWhereSeparateYawExtractionIsSensitive) {
  const Eigen::Matrix3d reference = Z(-0.9) * Y(kPi / 2.0 - 0.02);
  const Eigen::Matrix3d actual = X(0.02) * reference;
  const double old_offset = std::atan2(actual(1, 0), actual(0, 0)) -
                            std::atan2(reference(1, 0), reference(0, 0));
  EXPECT_GT(std::abs(old_offset), 0.1);
  EXPECT_TRUE(AlignYaw(actual, reference).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
}

TEST(HeadingAlignmentTest, OffsetIsEquivariantToRobotWorldHeading) {
  const Eigen::Matrix3d reference = Z(0.8) * Y(kPi / 2.0) * X(-0.3);
  const Eigen::Matrix3d actual = Z(-1.1) * X(0.07) * reference;
  const Eigen::Matrix3d alignment = AlignYaw(actual, reference);
  for (double heading : {-2.8, 0.2, kPi}) {
    EXPECT_TRUE(AlignYaw(Z(heading) * actual, reference).isApprox(Z(heading) * alignment, 1e-12));
  }
}

TEST(HeadingAlignmentTest, RelativeOrientationObservationDoesNotDependOnWorldHeading) {
  const Eigen::Matrix3d reference_initial = Z(0.5) * Y(-kPi / 2.0);
  const Eigen::Matrix3d actual_initial = Z(2.2) * X(0.03) * reference_initial;
  const Eigen::Matrix3d current_reference = Z(0.2) * reference_initial * X(0.5);
  const Eigen::Matrix3d current_actual = Z(-0.3) * actual_initial * Y(0.1);
  const Eigen::Matrix3d alignment = AlignYaw(actual_initial, reference_initial);
  const Eigen::Matrix3d relative = current_actual.transpose() * alignment * current_reference;
  const Eigen::Matrix3d heading = Z(-2.5);
  const Eigen::Matrix3d rotated_alignment = AlignYaw(heading * actual_initial, reference_initial);
  const Eigen::Matrix3d rotated_relative =
      (heading * current_actual).transpose() * rotated_alignment * current_reference;
  EXPECT_TRUE(rotated_relative.isApprox(relative, 1e-12));
}

TEST(HeadingAlignmentTest, FixedOffsetDoesNotEraseSubsequentRealTurns) {
  const Eigen::Matrix3d reference = Y(kPi / 2.0);
  const Eigen::Matrix3d actual_initial = Z(0.7) * reference;
  const Eigen::Matrix3d alignment = AlignYaw(actual_initial, reference);
  const Eigen::Matrix3d initial_relative = actual_initial.transpose() * alignment * reference;
  const Eigen::Matrix3d turned_relative = (Z(0.4) * actual_initial).transpose() * alignment * reference;
  EXPECT_TRUE(initial_relative.isApprox(Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_FALSE(turned_relative.isApprox(initial_relative, 1e-3));
}

TEST(HeadingAlignmentTest, AmbiguousOppositeTiltHasDeterministicYawEquivariantFallback) {
  const std::array<Eigen::Matrix3d, 4> orientations = {
      X(kPi), Y(kPi), Z(0.4) * X(kPi), Z(-1.3) * Y(kPi)};
  for (const Eigen::Matrix3d& actual : orientations) {
    const Eigen::Matrix3d reference = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d alignment = AlignYaw(actual, reference);
    ExpectPureYaw(alignment);
    EXPECT_TRUE(AlignYaw(actual, reference).isApprox(alignment, 1e-12));
    for (double heading : {-2.1, 0.5, kPi}) {
      EXPECT_TRUE(AlignYaw(Z(heading) * actual, reference).isApprox(Z(heading) * alignment, 1e-12));
    }
  }
}

TEST(HeadingAlignmentTest, RejectsNonfiniteAndInvalidOrientations) {
  Eigen::Matrix3d invalid = Eigen::Matrix3d::Identity();
  invalid(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(AlignYaw(invalid, Eigen::Matrix3d::Identity()), std::invalid_argument);
  EXPECT_THROW(AlignYaw(Eigen::Matrix3d::Identity(), invalid), std::invalid_argument);
  EXPECT_THROW(AlignYaw(Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero()), std::invalid_argument);
}

}  // namespace
}  // namespace runner::motion_transition

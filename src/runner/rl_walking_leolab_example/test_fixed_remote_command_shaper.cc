#include "rl_walking_leolab_example/fixed_remote_command_shaper.h"

#include <gtest/gtest.h>

#include <limits>

namespace runner {
namespace {

FixedRemoteCommandShaper MakeShaper(double reversal_pause_sec = 0.1) {
  FixedRemoteCommandShaper shaper;
  shaper.Configure({
      .speed_pos = Eigen::Vector3d(0.8, 0.7, 1.0),
      .speed_neg = Eigen::Vector3d(0.6, 0.5, 0.9),
      .activation_threshold = 0.2,
      .release_threshold = 0.12,
      .translation_axis_switch_margin = 0.1,
      .reversal_pause_sec = reversal_pause_sec,
      .control_dt = 0.02,
  });
  return shaper;
}

TEST(FixedRemoteCommandShaperTest, SelectsOneTranslationAxisAndKeepsYawIndependent) {
  auto shaper = MakeShaper();

  EXPECT_TRUE(
      shaper.Update(Eigen::Vector3d(0.9, 0.7, -0.8)).isApprox(Eigen::Vector3d(0.8, 0.0, -0.9)));
}

TEST(FixedRemoteCommandShaperTest, UsesActivationReleaseAndExactZero) {
  auto shaper = MakeShaper();

  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.19, 0.0, 0.19)).isZero(0.0));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.21, 0.0, 0.21)).isApprox(Eigen::Vector3d(0.8, 0.0, 1.0)));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.13, 0.0, 0.13)).isApprox(Eigen::Vector3d(0.8, 0.0, 1.0)));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.11, 0.0, 0.11)).isZero(0.0));
}

TEST(FixedRemoteCommandShaperTest, UsesMarginBeforeSwitchingTranslationAxis) {
  auto shaper = MakeShaper();

  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.6, 0.4, 0.0)).isApprox(Eigen::Vector3d(0.8, 0.0, 0.0)));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.45, 0.5, 0.0)).isApprox(Eigen::Vector3d(0.8, 0.0, 0.0)));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.3, 0.5, 0.0)).isApprox(Eigen::Vector3d(0.0, 0.7, 0.0)));
}

TEST(FixedRemoteCommandShaperTest, PausesOnlyTheReversingChannel) {
  auto shaper = MakeShaper(0.06);

  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.8, 0.0, 0.8)).isApprox(Eigen::Vector3d(0.8, 0.0, 1.0)));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(-0.8, 0.0, 0.8)).isApprox(Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(-0.8, 0.0, 0.8)).isApprox(Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(-0.8, 0.0, 0.8)).isApprox(Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(-0.8, 0.0, 0.8)).isApprox(Eigen::Vector3d(-0.6, 0.0, 1.0)));
}

TEST(FixedRemoteCommandShaperTest, NeutralTimeCountsTowardReversalPause) {
  auto shaper = MakeShaper(0.06);

  EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(0.8, 0.0, 0.0)).x(), 0.8);
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d::Zero()).isZero(0.0));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d::Zero()).isZero(0.0));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d::Zero()).isZero(0.0));
  EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(-0.8, 0.0, 0.0)).x(), -0.6);
}

TEST(FixedRemoteCommandShaperTest, YawReversalPauseDoesNotStopTranslation) {
  auto shaper = MakeShaper(0.06);

  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.8, 0.0, 0.8)).isApprox(Eigen::Vector3d(0.8, 0.0, 1.0)));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.8, 0.0, -0.8)).isApprox(Eigen::Vector3d(0.8, 0.0, 0.0)));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.8, 0.0, -0.8)).isApprox(Eigen::Vector3d(0.8, 0.0, 0.0)));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.8, 0.0, -0.8)).isApprox(Eigen::Vector3d(0.8, 0.0, 0.0)));
  EXPECT_TRUE(
      shaper.Update(Eigen::Vector3d(0.8, 0.0, -0.8)).isApprox(Eigen::Vector3d(0.8, 0.0, -0.9)));
}

TEST(FixedRemoteCommandShaperTest, NonFiniteInputStopsAndResetsTheShaper) {
  auto shaper = MakeShaper();

  EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(0.8, 0.0, 0.0)).x(), 0.8);
  EXPECT_TRUE(
      shaper.Update(Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)).isZero(0.0));
  EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(-0.8, 0.0, 0.0)).x(), -0.6);
}

}  // namespace
}  // namespace runner

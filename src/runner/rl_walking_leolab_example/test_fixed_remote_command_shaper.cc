#include "rl_walking_leolab_example/fixed_remote_command_shaper.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace runner {
namespace {

FixedRemoteCommandShaper MakeShaper(double reversal_pause_sec = 0.1, double activation_debounce_sec = 0.0) {
  FixedRemoteCommandShaper shaper;
  shaper.Configure({
      .speed_pos = Eigen::Vector3d(0.8, 0.7, 1.0),
      .speed_neg = Eigen::Vector3d(0.6, 0.5, 0.9),
      .activation_threshold = 0.2,
      .release_threshold = 0.12,
      .activation_debounce_sec = activation_debounce_sec,
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

TEST(FixedRemoteCommandShaperTest, RequiresStableStickActivationButReleasesImmediately) {
  auto shaper = MakeShaper(0.1, 0.04);

  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.8, 0.0, 0.0)).isZero(0.0));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d::Zero()).isZero(0.0));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.8, 0.0, 0.0)).isZero(0.0));
  EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(0.8, 0.0, 0.0)).x(), 0.8);
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d::Zero()).isZero(0.0));
}

TEST(FixedRemoteCommandShaperTest, DebouncesYawActivationIndependently) {
  auto shaper = MakeShaper(0.1, 0.04);

  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.0, 0.0, 0.8)).isZero(0.0));
  EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(0.0, 0.0, 0.8)).z(), 1.0);
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d::Zero()).isZero(0.0));
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

FixedRemoteCommandShaper MakeTranslationShaper(bool proportional = false, double debounce = 0.0,
                                              double min_speed = 0.0) {
  FixedRemoteCommandShaper shaper;
  shaper.Configure({
      .speed_pos = Eigen::Vector3d(0.8, 0.7, 1.0),
      .speed_neg = Eigen::Vector3d(0.6, 0.5, 0.9),
      .activation_debounce_sec = debounce,
      .translation_proportional = proportional,
      .translation_slew_enabled = true,
      .translation_acceleration = 1.0,
      .translation_deceleration = 2.0,
      .translation_min_speed = min_speed,
  });
  return shaper;
}

TEST(FixedRemoteCommandShaperTest, TranslationStartsWithBoundedAccelerationAndReachesExistingSpeed) {
  auto shaper = MakeTranslationShaper();
  double previous = 0.0;
  for (int i = 0; i < 40; ++i) {
    const auto command = shaper.Update(Eigen::Vector3d(1.0, 0.0, 0.0));
    EXPECT_GE(command.x(), previous);
    EXPECT_LE(command.x() - previous, 0.02 + 1e-12);
    EXPECT_DOUBLE_EQ(command.y(), 0.0);
    previous = command.x();
  }
  EXPECT_DOUBLE_EQ(previous, 0.8);
  EXPECT_DOUBLE_EQ(shaper.TargetCommand().x(), 0.8);
}

FixedRemoteCommandShaper MakeFastFixedShaper(bool terrain) {
  FixedRemoteCommandShaper shaper;
  EXPECT_TRUE(shaper.Configure({
      .speed_pos = terrain ? Eigen::Vector3d(1.0, 0.8, 1.5) : Eigen::Vector3d(0.5, 0.5, 1.0),
      .speed_neg = terrain ? Eigen::Vector3d(0.8, 0.8, 1.5) : Eigen::Vector3d(0.5, 0.5, 1.0),
      .activation_debounce_sec = 0.04,
      .translation_proportional = false,
      .translation_slew_enabled = true,
      .translation_acceleration = 5.0,
      .translation_deceleration = 2.0,
  }));
  return shaper;
}

TEST(FixedRemoteCommandShaperTest, FastFixedTranslationUsesSameTargetForLightAndFullSticks) {
  for (bool terrain : {false, true}) {
    for (int axis : {0, 1}) {
      for (double sign : {-1.0, 1.0}) {
        for (double magnitude : {0.21, 1.0}) {
          auto shaper = MakeFastFixedShaper(terrain);
          Eigen::Vector3d stick = Eigen::Vector3d::Zero();
          stick(axis) = sign * magnitude;
          const double target = terrain ? (axis == 0 && sign > 0.0 ? 1.0 : 0.8) : 0.5;
          EXPECT_TRUE(shaper.Update(stick).isZero(0.0));  // Existing activation confirmation.
          for (int step = 1; step <= 10; ++step) {
            const auto output = shaper.Update(stick);
            EXPECT_NEAR(output(axis), sign * std::min(target, step * 0.1), 1e-12);
            EXPECT_DOUBLE_EQ(output(1 - axis), 0.0);
            EXPECT_DOUBLE_EQ(shaper.TargetCommand()(axis), sign * target);
          }
        }
      }
    }
  }
}

TEST(FixedRemoteCommandShaperTest, FastStartupRetainsOriginalBrakingRateAndIndependentYaw) {
  for (bool terrain : {false, true}) {
    auto shaper = MakeFastFixedShaper(terrain);
    const double target = terrain ? 1.0 : 0.5;
    const double yaw = terrain ? 1.5 : 1.0;
    const Eigen::Vector3d stick(1.0, 0.0, 1.0);
    EXPECT_TRUE(shaper.Update(stick).isZero(0.0));
    for (int step = 1; step <= 10; ++step) {
      EXPECT_DOUBLE_EQ(shaper.Update(stick).z(), yaw);  // Yaw does not ramp.
    }
    const int braking_steps = static_cast<int>(std::ceil(target / 0.04));
    for (int step = 1; step <= braking_steps; ++step) {
      const auto output = shaper.Update(Eigen::Vector3d::Zero());
      EXPECT_NEAR(output.x(), std::max(0.0, target - step * 0.04), 1e-12);
      EXPECT_DOUBLE_EQ(output.z(), 0.0);  // Yaw still releases immediately.
    }
    EXPECT_TRUE(shaper.Update(Eigen::Vector3d::Zero()).isZero(0.0));
  }
}

TEST(FixedRemoteCommandShaperTest, ReleaseBrakesMonotonicallyToExactZeroInFiniteTime) {
  auto shaper = MakeTranslationShaper();
  for (int i = 0; i < 40; ++i) shaper.Update(Eigen::Vector3d(1.0, 0.0, 0.0));
  double previous = 0.8;
  for (int i = 0; i < 20; ++i) {
    const auto command = shaper.Update(Eigen::Vector3d::Zero());
    EXPECT_LE(command.x(), previous);
    EXPECT_GE(command.x(), 0.0);
    EXPECT_LE(previous - command.x(), 0.04 + 1e-12);
    EXPECT_TRUE(shaper.TargetCommand().isZero(0.0));
    previous = command.x();
  }
  EXPECT_DOUBLE_EQ(previous, 0.0);
  for (int i = 0; i < 50; ++i) EXPECT_TRUE(shaper.Update(Eigen::Vector3d::Zero()).isZero(0.0));
}

TEST(FixedRemoteCommandShaperTest, ProportionalTranslationRemapsDeadzoneAndRespectsAsymmetricMaxima) {
  auto shaper = MakeTranslationShaper(true);
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.2, 0.0, 0.0)).isZero(0.0));
  shaper.Update(Eigen::Vector3d(0.21, 0.0, 0.0));
  EXPECT_NEAR(shaper.TargetCommand().x(), 0.01, 1e-12);
  for (int i = 0; i < 40; ++i) shaper.Update(Eigen::Vector3d(0.6, 0.0, 0.0));
  EXPECT_NEAR(shaper.Update(Eigen::Vector3d(0.6, 0.0, 0.0)).x(), 0.4, 1e-12);
  shaper.Reset();
  for (int i = 0; i < 40; ++i) shaper.Update(Eigen::Vector3d(-1.2, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(-1.2, 0.0, 0.0)).x(), -0.6);
  shaper.Reset();
  for (int i = 0; i < 40; ++i) shaper.Update(Eigen::Vector3d(0.0, -0.6, 0.0));
  EXPECT_NEAR(shaper.Update(Eigen::Vector3d(0.0, -0.6, 0.0)).y(), -0.25, 1e-12);
}

TEST(FixedRemoteCommandShaperTest, ShortTapIsSmallAndDebounceStillRejectsSingleSampleSpikes) {
  auto shaper = MakeTranslationShaper(true, 0.04);
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(1.0, 0.0, 0.0)).isZero(0.0));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d::Zero()).isZero(0.0));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(1.0, 0.0, 0.0)).isZero(0.0));
  EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(1.0, 0.0, 0.0)).x(), 0.02);
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d::Zero()).isZero(0.0));
}

TEST(FixedRemoteCommandShaperTest, NonzeroTargetFloorDoesNotJumpTheShapedCommand) {
  auto shaper = MakeTranslationShaper(true, 0.0, 0.5);
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(0.2, 0.0, 0.0)).isZero(0.0));
  EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(0.21, 0.0, 0.0)).x(), 0.02);
  EXPECT_NEAR(shaper.TargetCommand().x(), 0.50375, 1e-12);
  for (int i = 0; i < 40; ++i) shaper.Update(Eigen::Vector3d(0.21, 0.0, 0.0));
  EXPECT_NEAR(shaper.Update(Eigen::Vector3d(0.21, 0.0, 0.0)).x(), 0.50375, 1e-12);
  for (int i = 0; i < 40; ++i) shaper.Update(Eigen::Vector3d(0.6, 0.0, 0.0));
  EXPECT_NEAR(shaper.TargetCommand().x(), 0.65, 1e-12);
  // The target returns to zero inside the deadzone; the output still brakes.
  EXPECT_GT(shaper.Update(Eigen::Vector3d(0.19, 0.0, 0.0)).x(), 0.0);
  EXPECT_TRUE(shaper.TargetCommand().isZero(0.0));
  for (int i = 0; i < 40; ++i) shaper.Update(Eigen::Vector3d::Zero());
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d::Zero()).isZero(0.0));
  for (int i = 0; i < 40; ++i) shaper.Update(Eigen::Vector3d(0.0, -0.21, 0.0));
  EXPECT_DOUBLE_EQ(shaper.TargetCommand().y(), -0.5);
}

TEST(FixedRemoteCommandShaperTest, ProportionalModeRequiresSlewEvenWithZeroMinimum) {
  for (double min_speed : {0.0, 0.5}) {
    auto shaper = MakeTranslationShaper();
    EXPECT_GT(shaper.Update(Eigen::Vector3d(1.0, 0.0, 0.0)).x(), 0.0);
    EXPECT_FALSE(shaper.Configure({.translation_proportional = true,
                                  .translation_slew_enabled = false,
                                  .translation_min_speed = min_speed}));
    EXPECT_TRUE(shaper.TargetCommand().isZero(0.0));
    EXPECT_TRUE(shaper.Update(Eigen::Vector3d::Zero()).isZero(0.0));
  }
  FixedRemoteCommandShaper shaper;
  EXPECT_TRUE(shaper.Configure({.translation_proportional = false, .translation_slew_enabled = false}));
  EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(1.0, 0.0, 0.0)).x(), 1.0);
  EXPECT_TRUE(shaper.Configure({.translation_proportional = true, .translation_slew_enabled = true}));
  EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(1.0, 0.0, 0.0)).x(), 0.02);
}

TEST(FixedRemoteCommandShaperTest, ReversalBrakesBeforeCountingTheZeroPause) {
  auto shaper = MakeTranslationShaper();
  for (int i = 0; i < 40; ++i) shaper.Update(Eigen::Vector3d(1.0, 0.0, 0.0));
  for (int i = 0; i < 20; ++i) {
    EXPECT_NEAR(shaper.Update(Eigen::Vector3d(-1.0, 0.0, 0.0)).x(), 0.8 - (i + 1) * 0.04, 1e-12);
  }
  for (int i = 0; i < 5; ++i) {
    EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(-1.0, 0.0, 0.0)).x(), 0.0);
  }
  EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(-1.0, 0.0, 0.0)).x(), -0.02);
}

TEST(FixedRemoteCommandShaperTest, AxisSwitchBrakesBeforeStartingTheOtherAxis) {
  auto shaper = MakeTranslationShaper();
  for (int i = 0; i < 40; ++i) shaper.Update(Eigen::Vector3d(1.0, 0.0, 0.0));
  for (int i = 0; i < 20; ++i) {
    const auto command = shaper.Update(Eigen::Vector3d(0.0, 1.0, 0.0));
    EXPECT_NEAR(command.x(), 0.8 - (i + 1) * 0.04, 1e-12);
    EXPECT_DOUBLE_EQ(command.y(), 0.0);
  }
  EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(0.0, 1.0, 0.0)).y(), 0.02);
}

TEST(FixedRemoteCommandShaperTest, ReturningToOriginalDirectionCancelsPendingReversal) {
  auto shaper = MakeTranslationShaper();
  for (int i = 0; i < 40; ++i) shaper.Update(Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_NEAR(shaper.Update(Eigen::Vector3d(-1.0, 0.0, 0.0)).x(), 0.76, 1e-12);
  EXPECT_NEAR(shaper.Update(Eigen::Vector3d(1.0, 0.0, 0.0)).x(), 0.78, 1e-12);
}

TEST(FixedRemoteCommandShaperTest, ReentryDiscardsOldTranslationAndStartsHeldInputFromZero) {
  auto shaper = MakeTranslationShaper(true, 0.04);
  for (int i = 0; i < 50; ++i) shaper.Update(Eigen::Vector3d(1.0, 0.0, 1.0));
  shaper.Reset();
  EXPECT_TRUE(shaper.TargetCommand().isZero(0.0));
  for (int i = 0; i < 100; ++i) EXPECT_TRUE(shaper.Update(Eigen::Vector3d::Zero()).isZero(0.0));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d(1.0, 0.0, 0.0)).isZero(0.0));
  EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(1.0, 0.0, 0.0)).x(), 0.02);
  // Configure is also used on every Enter, including switching Leo profiles.
  shaper.Configure({.translation_slew_enabled = true});
  EXPECT_TRUE(shaper.TargetCommand().isZero(0.0));
  EXPECT_TRUE(shaper.Update(Eigen::Vector3d::Zero()).isZero(0.0));
}

TEST(FixedRemoteCommandShaperTest, InputLossAndNonfiniteInputStopImmediatelyWithoutRampTail) {
  for (const bool lost : {true, false}) {
    auto shaper = MakeTranslationShaper();
    for (int i = 0; i < 40; ++i) shaper.Update(Eigen::Vector3d(1.0, 0.0, 1.0));
    const Eigen::Vector3d invalid = lost ? Eigen::Vector3d(1.0, 0.0, 1.0)
                                       : Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 0.0);
    EXPECT_TRUE(shaper.Update(invalid, !lost).isZero(0.0));
    EXPECT_TRUE(shaper.TargetCommand().isZero(0.0));
    EXPECT_DOUBLE_EQ(shaper.Update(Eigen::Vector3d(1.0, 0.0, 0.0)).x(), 0.02);
  }
}

TEST(FixedRemoteCommandShaperTest, YawMatchesLegacyExactlyForMixedInputsIncludingReversals) {
  auto legacy = MakeShaper(0.1, 0.04);
  auto proportional = MakeTranslationShaper(true, 0.04);
  auto fixed = MakeTranslationShaper(false, 0.04);
  std::mt19937 random(42);
  std::uniform_real_distribution<double> stick(-1.0, 1.0);
  for (int i = 0; i < 500; ++i) {
    const Eigen::Vector3d raw(stick(random), stick(random), stick(random));
    for (int frame = 0; frame < 10; ++frame) {
      const double yaw = legacy.Update(raw).z();
      const auto a = proportional.Update(raw);
      const auto b = fixed.Update(raw);
      EXPECT_DOUBLE_EQ(a.z(), yaw);
      EXPECT_DOUBLE_EQ(b.z(), yaw);
      EXPECT_TRUE(a.x() == 0.0 || a.y() == 0.0);
      EXPECT_TRUE(b.x() == 0.0 || b.y() == 0.0);
    }
  }
}

}  // namespace
}  // namespace runner

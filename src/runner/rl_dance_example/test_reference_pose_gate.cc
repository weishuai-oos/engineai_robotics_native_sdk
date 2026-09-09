#include "rl_dance_example/reference_pose_gate.h"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace runner {
namespace {

TEST(ReferencePoseGateTest, CountsOnlyConsecutiveInToleranceCycles) {
  ReferencePoseGate gate;
  ASSERT_TRUE(gate.Configure(true, 0.12, 3));
  EXPECT_TRUE(gate.IsActive());
  EXPECT_TRUE(gate.Observe(false, 0.10));
  EXPECT_TRUE(gate.Observe(false, 0.11));
  EXPECT_FALSE(gate.Observe(false, 0.12));
}

TEST(ReferencePoseGateTest, OutOfToleranceResetsConsecutiveCount) {
  ReferencePoseGate gate;
  ASSERT_TRUE(gate.Configure(true, 0.12, 3));
  EXPECT_TRUE(gate.Observe(false, 0.10));
  EXPECT_TRUE(gate.Observe(false, 0.20));
  EXPECT_EQ(gate.settle_cycles(), 0);
  EXPECT_TRUE(gate.Observe(false, 0.10));
  EXPECT_TRUE(gate.Observe(false, 0.10));
  EXPECT_FALSE(gate.Observe(false, 0.10));
}

TEST(ReferencePoseGateTest, NonFiniteErrorNeverPasses) {
  ReferencePoseGate gate;
  ASSERT_TRUE(gate.Configure(true, 0.12, 1));
  EXPECT_TRUE(gate.Observe(false, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(gate.settle_cycles(), 0);
  EXPECT_TRUE(gate.Observe(false, std::numeric_limits<double>::infinity()));
  EXPECT_EQ(gate.settle_cycles(), 0);
}

TEST(ReferencePoseGateTest, DisabledModeDoesNotChangeBehavior) {
  ReferencePoseGate gate;
  ASSERT_TRUE(gate.Configure(false, 0.12, 3));
  EXPECT_FALSE(gate.IsActive());
  EXPECT_FALSE(gate.Observe(false, 0.0));
  EXPECT_EQ(gate.settle_cycles(), 0);
}

TEST(ReferencePoseGateTest, BridgeCyclesDoNotCount) {
  ReferencePoseGate gate;
  ASSERT_TRUE(gate.Configure(true, 0.12, 2));
  EXPECT_TRUE(gate.Observe(true, 0.0));
  EXPECT_EQ(gate.settle_cycles(), 0);
  EXPECT_TRUE(gate.Observe(true, 0.0));
  EXPECT_EQ(gate.settle_cycles(), 0);
  EXPECT_TRUE(gate.Observe(false, 0.0));
  EXPECT_FALSE(gate.Observe(false, 0.0));
}

TEST(ReferencePoseGateTest, RejectsInvalidConfiguration) {
  ReferencePoseGate gate;
  EXPECT_FALSE(gate.Configure(true, 0.0, 3));
  EXPECT_FALSE(gate.Configure(true, std::numeric_limits<double>::quiet_NaN(), 3));
  EXPECT_FALSE(gate.Configure(true, 0.12, 0));
}

}  // namespace
}  // namespace runner

#include <gtest/gtest.h>
#include <cnpy.h>
#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <unistd.h>

#include "rl_supermimic/flat_policy.h"
#include "rl_supermimic/policy_contract.h"
#include "rl_supermimic/reference.h"

namespace runner::supermimic {
namespace {

TEST(SupermimicContract, HistoryIsFrameMajorAndOldestToNewest) {
  History history;
  Observation observation;
  for (int i = 0; i < kObservation; ++i) observation(i) = i;
  PushObservation(observation, true, &history);
  for (int row = 0; row < kHistory; ++row) EXPECT_EQ(history.row(row), observation.transpose());
  for (int frame = 1; frame <= kHistory; ++frame) {
    Observation next = observation.array() + 100 * frame;
    PushObservation(next, false, &history);
  }
  Eigen::VectorXf goal = Eigen::VectorXf::LinSpaced(kGoal, -20, 20);
  const Input input = BuildInput(history, goal);
  for (int frame = 0; frame < kHistory; ++frame)
    for (int i = 0; i < kObservation; ++i)
      EXPECT_EQ(input(frame * kObservation + i), i + 100 * (frame + 1));
  EXPECT_EQ(input.tail<kGoal>(), goal);
}

TEST(SupermimicContract, ObservationHasExactLayoutAndRowMajorOrientation) {
  Eigen::VectorXd q = Eigen::VectorXd::Constant(kActions, 2.0);
  Eigen::VectorXd qd = Eigen::VectorXd::Constant(kActions, 3.0);
  Eigen::VectorXd defaults = Eigen::VectorXd::Constant(kActions, 0.5);
  Eigen::VectorXd previous = Eigen::VectorXd::Constant(kActions, 4.0);
  const Eigen::Matrix3d actual = Eigen::AngleAxisd(0.43, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d reference = Eigen::AngleAxisd(-0.7, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d angular(5, 6, 7);
  const auto obs = BuildObservation(q, qd, actual, angular, reference, defaults, previous);
  EXPECT_TRUE(obs.segment<25>(0).isConstant(1.5f));
  EXPECT_TRUE(obs.segment<25>(25).isConstant(3.0f));
  EXPECT_EQ(obs.segment<3>(50), angular.cast<float>());
  EXPECT_EQ(obs.segment<3>(53), Eigen::Vector3f(0, 0, -1));
  EXPECT_TRUE(obs.segment<25>(56).isConstant(4.0f));
  const Eigen::Matrix3d relative = actual.transpose() * reference;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 2; ++col)
      EXPECT_FLOAT_EQ(obs(81 + 2 * row + col), static_cast<float>(relative(row, col)));
}

TEST(SupermimicContract, HeadingAlignmentIsInvariantForOppositeDuelHeading) {
  const auto yaw = Eigen::AngleAxisd(2.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto actual = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const auto reference0 = Eigen::AngleAxisd(-1.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d reference =
      reference0 * Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Matrix3d aligned = AlignReferenceYaw(actual, reference0, reference);
  const Eigen::Matrix3d rotated = AlignReferenceYaw(yaw * actual, reference0, reference);
  EXPECT_TRUE(rotated.isApprox(yaw * aligned, 1e-12));
  const Eigen::VectorXd zeros = Eigen::VectorXd::Zero(kActions);
  const auto first = BuildObservation(zeros, zeros, actual, Eigen::Vector3d::Zero(), aligned, zeros, zeros);
  const auto second = BuildObservation(zeros, zeros, yaw * actual, Eigen::Vector3d::Zero(), rotated, zeros, zeros);
  EXPECT_TRUE(first.isApprox(second, 1e-6));
}

TEST(SupermimicContract, ActionIsScaledAroundDefaultRatherThanReference) {
  const auto defaults = Eigen::VectorXd::Constant(kActions, 0.5);
  const auto scales = Eigen::VectorXd::Constant(kActions, 0.2);
  const auto action = Eigen::VectorXd::Constant(kActions, -2.0);
  EXPECT_TRUE(ActionTarget(defaults, scales, action).isConstant(0.1, 1e-12));
}

TEST(SupermimicContract, SequenceUsesPenultimateFrameThroughoutClosedLoopHold) {
  Sequence sequence;
  ASSERT_TRUE(sequence.Configure(5, 0.06, 0.02));
  for (int step = 0; step < 7; ++step) {
    EXPECT_EQ(sequence.frame(), std::min(step, 3));
    EXPECT_FALSE(sequence.finished());
    sequence.Advance();
  }
  EXPECT_TRUE(sequence.finished());
  sequence.Reset();
  EXPECT_EQ(sequence.frame(), 0);
  EXPECT_FALSE(sequence.finished());
  EXPECT_FALSE(sequence.Configure(1, 0, .02));
  EXPECT_FALSE(sequence.Configure(5, -1, .02));
  EXPECT_FALSE(sequence.Configure(5, 1, std::numeric_limits<double>::quiet_NaN()));
}

TEST(SupermimicContract, SkippingStaticTailMatchesTheOriginalDynamicClipTiming) {
  Sequence original;
  Sequence numbered;
  ASSERT_TRUE(original.Configure(99, 0, .02));
  ASSERT_TRUE(numbered.Configure(199, 0, .02, 100));
  for (int step = 0; step < 98; ++step) {
    EXPECT_FALSE(numbered.finished());
    EXPECT_EQ(numbered.frame(), original.frame());
    EXPECT_LT(numbered.frame(), 99);  // The appended static tail starts at 99.
    original.Advance();
    numbered.Advance();
  }
  EXPECT_TRUE(original.finished());
  EXPECT_TRUE(numbered.finished());
  numbered.Reset();
  EXPECT_FALSE(numbered.finished());
  EXPECT_EQ(numbered.frame(), 0);
}

TEST(SupermimicContract, SkippingStaticTailValidatesBoundsAndKeepsDefaultPlayback) {
  Sequence legacy;
  Sequence explicit_zero;
  ASSERT_TRUE(legacy.Configure(120, 0, .02));
  ASSERT_TRUE(explicit_zero.Configure(120, 0, .02, 0));
  for (int step = 0; step < 119; ++step) {
    EXPECT_FALSE(explicit_zero.finished());
    EXPECT_EQ(explicit_zero.frame(), legacy.frame());
    explicit_zero.Advance();
    legacy.Advance();
  }
  EXPECT_TRUE(explicit_zero.finished());
  EXPECT_TRUE(legacy.finished());
  EXPECT_FALSE(legacy.Configure(5, 0, .02, -1));
  EXPECT_FALSE(legacy.Configure(5, 0, .02, 4));
  EXPECT_FALSE(legacy.Configure(5, 0, .02, 100));
  ASSERT_TRUE(legacy.Configure(5, 0, .02, 3));
  EXPECT_EQ(legacy.frame(), 0);
  legacy.Advance();
  EXPECT_TRUE(legacy.finished());
}

TEST(SupermimicContract, NonfiniteOrWrongSizedValuesAreRejected) {
  History history = History::Zero();
  Eigen::VectorXf goal = Eigen::VectorXf::Zero(kGoal);
  goal(0) = std::numeric_limits<float>::infinity();
  EXPECT_THROW(BuildInput(history, goal), std::invalid_argument);
  EXPECT_THROW(BuildInput(history, Eigen::VectorXf::Zero(1)), std::invalid_argument);
  EXPECT_THROW(ActionTarget(Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(kActions),
                            Eigen::VectorXd::Zero(kActions)), std::invalid_argument);
}

TEST(SupermimicReference, Int32PayloadIsRejectedBeforeReinterpretingAsFloat32) {
  const auto path = std::filesystem::temp_directory_path() /
                    ("engineai-supermimic-dtype-" + std::to_string(getpid()) + ".npz");
  const int32_t values[3] = {1, 2, 3};
  cnpy::npz_save(path.string(), "root_pos", values, {1, 3}, "w");
  EXPECT_THROW(LoadReference(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(SupermimicReference, ExportedReferenceHasValidRootStateAndGoal) {
  const char* package = std::getenv("ENGINEAI_SUPERMIMIC_PACKAGE");
  if (!package) GTEST_SKIP() << "Set ENGINEAI_SUPERMIMIC_PACKAGE to validate an exported package";
  const auto reference = LoadReference((std::filesystem::path(package) / "trajectories/supermimic_reference.npz").string());
  EXPECT_GE(reference.frames(), 2);
  EXPECT_EQ(reference.goals.rows(), reference.frames());
  EXPECT_EQ(reference.goals.cols(), kGoal);
  EXPECT_TRUE(reference.goals.allFinite());
  const auto arrays = cnpy::npz_load((std::filesystem::path(package) /
                                     "trajectories/supermimic_reference.npz").string());
  const Eigen::VectorXd first_pose = Eigen::Map<const Eigen::VectorXf>(
      arrays.at("joint_pos").data<float>(), kActions).cast<double>();
  ASSERT_EQ(reference.initial_joint_pos.size(), kActions);
  EXPECT_TRUE(reference.initial_joint_pos.isApprox(first_pose, 1e-12));
}

TEST(SupermimicPolicy, NativeMnnRunsDeterministicallyOnReferenceStates) {
  const char* package = std::getenv("ENGINEAI_SUPERMIMIC_PACKAGE");
  if (!package) GTEST_SKIP() << "Set ENGINEAI_SUPERMIMIC_PACKAGE to validate native MNN inference";
  const std::filesystem::path root(package);
  const auto config = YAML::LoadFile((root / "default.yaml").string());
  const auto reference_path = root / config["reference_file"].as<std::string>();
  const auto reference = LoadReference(reference_path.string());
  const auto arrays = cnpy::npz_load(reference_path.string());
  const auto defaults = config["default_joint_pos"].as<std::vector<double>>();
  const auto scales = config["action_scale"].as<std::vector<double>>();
  ASSERT_EQ(defaults.size(), kActions);
  ASSERT_EQ(scales.size(), kActions);
  const Eigen::VectorXd default_q = Eigen::Map<const Eigen::VectorXd>(defaults.data(), kActions);
  const Eigen::VectorXd action_scale = Eigen::Map<const Eigen::VectorXd>(scales.data(), kActions);
  FlatPolicy policy((root / config["policy_file"].as<std::string>()).string());
  History history = History::Zero();
  Eigen::VectorXd previous_action = Eigen::VectorXd::Zero(kActions);
  bool first = true;
  const int frames[] = {0, reference.frames() / 2, reference.frames() - 2};
  for (const int frame : frames) {
    SCOPED_TRACE(frame);
    const Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXf>(
        arrays.at("joint_pos").data<float>() + frame * kActions, kActions).cast<double>();
    const Eigen::VectorXd qd = Eigen::Map<const Eigen::VectorXf>(
        arrays.at("joint_vel").data<float>() + frame * kActions, kActions).cast<double>();
    const Eigen::Vector3d angular_velocity_world = Eigen::Map<const Eigen::Vector3f>(
        arrays.at("root_ang_vel").data<float>() + frame * 3).cast<double>();
    const auto& rotation = reference.rotations[frame];
    const auto observation = BuildObservation(q, qd, rotation, rotation.transpose() * angular_velocity_world,
                                              rotation, default_q, previous_action);
    PushObservation(observation, first, &history);
    first = false;
    const Input input = BuildInput(history, reference.goals.row(frame).transpose());
    Eigen::VectorXd output;
    ASSERT_TRUE(policy.Infer(input, &output));
    ASSERT_EQ(output.size(), kActions);
    ASSERT_TRUE(output.allFinite());
    Eigen::VectorXd repeated;
    ASSERT_TRUE(policy.Infer(input, &repeated));
    EXPECT_TRUE(output.isApprox(repeated, 1e-6));
    EXPECT_TRUE(ActionTarget(default_q, action_scale, output).allFinite());
    previous_action = output;
  }
}

TEST(SupermimicPolicy, LargeFiniteInputsPreserveTheExportedActionContract) {
  const char* package = std::getenv("ENGINEAI_SUPERMIMIC_PACKAGE");
  if (!package) GTEST_SKIP() << "Set ENGINEAI_SUPERMIMIC_PACKAGE to validate exported action bounds";
  const std::filesystem::path root(package);
  const auto config = YAML::LoadFile((root / "default.yaml").string());
  FlatPolicy policy((root / "policies/policy.mnn").string());
  Eigen::VectorXd output;
  // Synthetic inference input only; never send this stress sample to a robot.
  ASSERT_TRUE(policy.Infer(Eigen::VectorXf::Constant(kInput, 100.0f), &output));
  ASSERT_TRUE(output.allFinite());
  const auto formula = config["action_target_formula"].as<std::string>();
  if (formula == "default_joint_pos + action_scale * raw_action") {
    // The prone export produces a finite output above 20 for this sample.
    // Rejecting or clamping it changes both the target and subsequent history.
    EXPECT_GT(output.cwiseAbs().maxCoeff(), 20.0);
  } else {
    ASSERT_EQ(formula, "default_joint_pos + action_scale * action_output");
    EXPECT_LE(output.cwiseAbs().maxCoeff(), 20.0001);
  }
  EXPECT_EQ(ActionTarget(Eigen::VectorXd::Zero(kActions), Eigen::VectorXd::Ones(kActions), output), output);
}

}  // namespace
}  // namespace runner::supermimic

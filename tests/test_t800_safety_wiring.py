import json
import pathlib
import unittest

import yaml


SDK_ROOT = pathlib.Path(__file__).resolve().parents[1]


def load_yaml(relative_path: str):
    with (SDK_ROOT / relative_path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class T800SafetyWiringTest(unittest.TestCase):
    def test_robot_gate_runs_before_transform_and_motor(self):
        config = load_yaml("assets/config/t800/task_resident/robot.yaml")
        motor_task = next(task for task in config["tasks"] if task["task"] == "motor")
        runners = [entry["name"] for entry in motor_task["runner"] if entry.get("enabled", True)]
        self.assertEqual(
            runners,
            [
                "t800_safety_runner",
                "joint_motor_transform_runner",
                "t800_motor_pre_driver_safety_runner",
                "motor_runner",
            ],
        )
        self.assertEqual(motor_task["period"], 0.002)

    def test_sim_gate_runs_before_publish(self):
        config = load_yaml("assets/config/t800/task_resident/sim.yaml")
        publish_task = next(task for task in config["tasks"] if task["task"] == "sim_publish")
        runners = [entry["name"] for entry in publish_task["runner"] if entry.get("enabled", True)]
        self.assertEqual(runners, ["t800_safety_runner", "sim_publish_runner"])
        self.assertEqual(publish_task["period"], 0.002)

    def test_post_gate_joint_override_is_disabled(self):
        config = load_yaml("assets/config/t800/joint_motor_transform_runner/default.yaml")
        self.assertIs(config["enable_joint_override_converter"], False)

    def test_hardware_motor_debug_is_freshness_gated(self):
        source = (
            SDK_ROOT / "src/runner/t800_safety/src/t800_safety.cc"
        ).read_text()
        header = (
            SDK_ROOT
            / "src/runner/t800_safety/include/t800_safety/t800_safety_runner.h"
        ).read_text()
        self.assertIn("motor_debug_subscriber_.Get()", source)
        self.assertIn("Subscriber<data::MotorDebug> motor_debug_subscriber_", header)
        self.assertIn("last_motor_debug_sample_", source)
        self.assertIn("require_motor_debug = !common::IsInMujoco()", source)
        self.assertIn("kMotorDebugFreshnessLimitNs", source)
        self.assertIn("T800MotorPreDriverSafetyRunner::Run", source)
        self.assertIn("joint_info.SetZeroCommand();\n  data_store_->joint_info.SetCommandWithoutTorque", source)
        self.assertIn("motor_info.SetZeroCommand();\n  data_store_->motor_info.SetCommandWithoutTorque", source)

    def test_selective_build_maps_both_safety_runners_to_one_sdk_module(self):
        runner_cmake = (SDK_ROOT / "src/runner/CMakeLists.txt").read_text()
        registry_cmake = (SDK_ROOT / "cmake/AutoRegisterRunners.cmake").read_text()
        self.assertIn("t800_motor_pre_driver_safety_runner", runner_cmake)
        self.assertIn('set(runner_base "t800_safety")', runner_cmake)
        self.assertIn("t800_motor_pre_driver_safety_runner", registry_cmake)
        self.assertIn('set(_runner_base "t800_safety")', registry_cmake)
        self.assertIn("file(GLOB_RECURSE t800_safety_sources", registry_cmake)
        self.assertIn('set(_t800_safety_target "src::runner::t800_safety")', registry_cmake)

    def test_motion_periods_use_supported_policy_cadences(self):
        config = load_yaml("assets/config/t800/task_motion/default.yaml")
        supported_periods = {0.002, 0.01, 0.02}
        for motion in config["tasks"]:
            self.assertIn(motion["period"], supported_periods, motion["motion"])
        getup = next(motion for motion in config["tasks"] if motion["motion"] == "getup")
        self.assertEqual(getup["period"], 0.02)

    def test_runtime_safety_has_no_host_repository_contract(self):
        contract = json.loads(
            (SDK_ROOT / "assets/config/t800/safety/t800_safety_contract.json").read_text()
        )
        self.assertNotIn("host", json.dumps(contract).lower())
        getup_param = (
            SDK_ROOT
            / "src/data/_param/rl_getup_example_param/rl_getup_example_param.h"
        ).read_text()
        self.assertNotIn("host_joint_names", getup_param)
        self.assertFalse((SDK_ROOT / "tools/sync_t800_safety_contract.py").exists())

    def test_leo_mapping_uses_sdk_local_policy_names(self):
        param_header = (
            SDK_ROOT
            / "src/data/_param/rl_walking_leolab_example_param/rl_walking_leolab_example_param.h"
        ).read_text()
        self.assertIn("policy_joint_names", param_header)
        self.assertNotIn("host_joint_names", param_header)
        for relative in (
            "assets/config/t800/rl_walking_leolab_example/default.yaml",
            "assets/config/t800/rl_walking_leolab_terrain_example/default.yaml",
            "assets/config/t800/rl_walking_leolab_terrain_example/profiles/std_offset/default.yaml",
        ):
            config = load_yaml(relative)
            self.assertEqual(len(config["policy_joint_names"]), config["num_actions"])
            self.assertEqual(len(config["joint_names"]), config["num_actions"])

    def test_policies_mask_failed_head_actions_before_observation_and_output(self):
        sources = (
            "src/runner/rl_walking_example/src/rl_walking_example_runner.cc",
            "src/runner/rl_walking_custom_example/src/rl_walking_custom_example_runner.cc",
            "src/runner/rl_lab/src/rl_lab_runner.cc",
            "src/runner/rl_walking_leolab_example/src/rl_walking_leolab_example_runner.cc",
            "src/runner/rl_dance_example/src/rl_dance_example_runner.cc",
            "src/runner/rl_mimic_trajectory_example/src/rl_mimic_trajectory_example_runner.cc",
        )
        for relative in sources:
            source = (SDK_ROOT / relative).read_text()
            self.assertGreaterEqual(source.count("MaskFailedHeadActions"), 2, relative)


if __name__ == "__main__":
    unittest.main()

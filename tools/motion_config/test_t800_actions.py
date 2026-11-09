"""Regressions for the T800 task graph and its deployed policy assets.

Uses the SDK's existing PyYAML and NumPy tooling; no robot or SDK libraries are
needed. Run with ``python3 -B -m unittest discover -s tools/motion_config -v``.
"""

from pathlib import Path
import hashlib
import json
import unittest

import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_ROOT = REPO_ROOT / "assets/config/t800"
REPLACED_TAGS = {"rl_dance_example", "rl_victory_example", "rl_ridicule", "rl_celebration"}
NEW_TAGS = {"rl_supermimic_come_here", "rl_supermimic_left_hook",
            "rl_supermimic_pull_hammer", "rl_supermimic_right_high_kick"}
ARCHIVED_DIRS = {
    "rl_dance_example", "rl_victory_example", "rl_celebration",
    "rl_ridicule_motion1_baseline_scratch_3090_j9829_m26500",
    "rl_prone_to_stance_hybrid_9825_m29500",
    "rl_walking_leolab_example",
}
TAUNT_BACKUP_PACKAGES = {
    "rl_supermimic_zq_come_here_02_j13904_1_m36000",
    "rl_supermimic_zq_come_here_03_j13904_2_m38000",
}
COMBINED_WALKING_PROFILE = "rl_walking_leolab_ankle_combined_v2_140000"
PRONE_PACKAGE = "rl_supermimic_prone_j12768_m22000"
SUPER_MOTIONS = {
    "supermimic_come_here": (
        "rl_supermimic_come_here", "B", "rl_supermimic_zq_come_here_01_j13904_0_m36000",
        "908333f3d1301e990455caac1f9eba600456874d6e9833d73d64b9ea0ede6088"),
    "supermimic_left_hook": (
        "rl_supermimic_left_hook", "Y", "rl_supermimic_zq_left_hook_02_j13564_m82000",
        "cf6ea7f7e87373c73d9613c4930d4e9ed0a2ffe5af4b70d16b8d06fb853977e2"),
    "supermimic_pull_hammer": (
        "rl_supermimic_pull_hammer", "X", "rl_supermimic_zq_pull_hammer_20261102_j13551_m182000",
        "319199d0863cd10e980af238fc4d1199f8e87541da88c9398dce9ddddffc99db"),
    "supermimic_right_high_kick": (
        "rl_supermimic_right_high_kick", "A", "rl_supermimic_zq_right_high_kick_20261102_j13551_m184000",
        "9d1e786eb567a8b957748d2d3711b5db2c89c1845c43c78cdf77ba1c9bb7bfa9"),
}
SUPER_EXPORTS = {directory: default_hash for _, _, directory, default_hash in SUPER_MOTIONS.values()}
SUPER_EXPORTS[PRONE_PACKAGE] = "1dc6526eed5a1a9be35f70a0de7619ca9e91418eefb89ed1fb7a51790eb0dc56"

# These values were captured before replacing the four RB+face-button motions.
# They cover the other fourteen motions after replacing the prone recovery too.
PRESERVED_MOTIONS = {
    "walk": (0.01, ["LB", "B"], "rl_walking_example_runner", "rl_walking_example"),
    "walk_custom": (0.01, ["LB", "Y"], "rl_walking_custom_example_runner", "rl_walking_custom_example"),
    "walk_leo": (0.02, ["LB", "A"], "rl_walking_leolab_example_runner", "rl_walking_leolab_example"),
    "leo_taunt": (0.02, ["CROSS_Y_LEFT", "CROSS_Y_RIGHT", "B"],
                  "rl_walking_leolab_example_runner", "rl_walking_leolab_taunt"),
    "walk_leo_terrain": (0.02, ["LB", "X"], "rl_walking_leolab_example_runner",
                         "rl_walking_leolab_terrain_example"),
    "rl_left_front_kick_002_complete_heightfall": (0.02, ["RB", "CROSS_X_UP"],
        "rl_dance_example_runner", "rl_left_front_kick_002_complete_heightfall"),
    "rl_roundhouse_kick": (0.02, ["RB", "CROSS_X_DOWN"],
        "rl_dance_example_runner", "rl_roundhouse_kick"),
    "rl_straight_punch_L_improved": (0.02, ["RB", "CROSS_Y_LEFT"],
        "rl_dance_example_runner", "rl_straight_punch_L_improved"),
    "rl_left_hook_001_improved": (0.02, ["CROSS_X_UP", "CROSS_X_DOWN", "CROSS_Y_RIGHT"],
        "rl_dance_example_runner", "rl_left_hook_001_improved"),
    "rl_punch_example": (0.02, ["RB", "CROSS_Y_RIGHT"], "rl_dance_example_runner", "rl_punch_example"),
    "supine_to_stance": (0.02, ["START", "CROSS_X_UP"], "rl_dance_example_runner", "rl_supine_to_stance"),
    "getup": (0.02, ["LB", "START"], "rl_getup_example_runner", "rl_getup_example"),
    "getup2": (0.02, ["LB", "BACK"], "rl_getup_example_runner", "rl_getup2_example"),
    "stance_to_supine": (0.01, ["START", "CROSS_X_DOWN"],
        "rl_mimic_trajectory_runner", "rl_stance_to_supine"),
}

PRESERVED_SIM_SCOPES = {
    "resident_task": "task_resident/sim",
    "motion_task": "task_motion/default",
    "global_options": "global_options/release",
    "lcm": "lcm/default",
    "google_logger": "google_logger/default",
    "motor": "motor/default",
    "model": "model/default",
    "base_state_estimator": "base_state_estimator/default",
    "motion_state_estimator": "motion_state_estimator/default",
    "parallel_ankle": "parallel_ankle/default",
    "joint_motor_transform": "joint_motor_transform_runner/default",
    "passive": "passive/default",
    "pd_stand": "pd_stand/default",
    "pd_stand_x": "pd_stand/pose_x",
    "pd_stand_y": "pd_stand/pose_y",
    "rl_walking_example": "rl_walking_example/default",
    "ros2_bridge": "ros2_bridge/default",
    "rl_supine_to_stance": "rl_supine_to_stance_v1_hybrid_9843_m13000/default",
    "rl_prone_to_stance": PRONE_PACKAGE + "/default",
    "rl_stance_to_supine": "rl_stance_to_supine/default",
    "rl_walking_custom_example": "rl_walking_custom_example/default",
    "rl_walking_leolab_example": COMBINED_WALKING_PROFILE + "/default",
    "rl_walking_leolab_taunt": "rl_walking_leolab_taunt/default",
    "rl_walking_leolab_terrain_example": "rl_walking_leolab_terrain_example/default",
    "rl_getup_example": "rl_getup_example/default",
    "rl_getup2_example": "rl_getup2_example/default",
    "rl_left_front_kick_002_complete_heightfall": "rl_left_front_kick_002_complete_heightfall/default",
    "rl_roundhouse_kick": "rl_roundhouse_kick/default",
    "rl_straight_punch_L_improved": "rl_straight_punch_L_improved/default",
    "rl_left_hook_001_improved": "rl_left_hook_001_improved/default",
    "rl_punch_example": "rl_punch_example/default",
}


def load_yaml(path):
    return yaml.safe_load(path.read_text())


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class T800ActionBaselineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mode = load_yaml(CONFIG_ROOT / "mode.yaml")
        cls.task_list = load_yaml(CONFIG_ROOT / "task_motion/default.yaml")["tasks"]
        cls.tasks = {task["motion"]: task for task in cls.task_list}

    def test_all_transition_targets_exist(self):
        self.assertEqual(len(self.tasks), len(self.task_list), "duplicate motion names")
        for name, task in self.tasks.items():
            targets = list(task.get("manual_transition", []))
            if task.get("auto_transition"):
                targets.append(task["auto_transition"])
            for target in targets:
                with self.subTest(source=name, target=target):
                    self.assertIn(target, self.tasks)

    def test_modes_keep_required_scopes_and_cover_task_parameter_tags(self):
        self.assertEqual(set(self.mode["mode"]), {"sim", "robot"})
        for name, entries in self.mode["mode"].items():
            scopes = {entry["tag"]: entry["scope"] for entry in entries}
            self.assertEqual(len(scopes), len(entries), f"duplicate tags in {name}")
            expected = dict(PRESERVED_SIM_SCOPES)
            if name == "robot":
                expected.update(resident_task="task_resident/robot", lcm="lcm/robot", pd_stand="pd_stand/robot")
            stable = {tag: scope for tag, scope in scopes.items() if tag not in REPLACED_TAGS | NEW_TAGS}
            self.assertEqual(stable, expected)
            for tag, scope in scopes.items():
                with self.subTest(mode=name, tag=tag):
                    self.assertTrue((CONFIG_ROOT / (scope + ".yaml")).is_file(), scope)
            for task in self.task_list:
                for runner in task.get("runner", []):
                    if runner.get("enabled") and "param_tag" in runner:
                        with self.subTest(mode=name, motion=task["motion"]):
                            self.assertIn(runner["param_tag"], scopes)

    def test_fourteen_other_motions_keep_keys_runners_tags_and_periods(self):
        self.assertEqual(len(PRESERVED_MOTIONS), 14)
        for name, expected in PRESERVED_MOTIONS.items():
            with self.subTest(motion=name):
                task = self.tasks[name]
                enabled = [runner for runner in task["runner"] if runner.get("enabled")]
                self.assertEqual(len(enabled), 1)
                actual = (task["period"], task["key"], enabled[0]["name"], enabled[0]["param_tag"])
                self.assertEqual(actual, expected)

    def test_active_policy_and_trajectory_paths_exist(self):
        for name, entries in self.mode["mode"].items():
            for entry in entries:
                config_path = CONFIG_ROOT / (entry["scope"] + ".yaml")
                config = load_yaml(config_path)
                # SuperMimic exports keep their package-relative asset paths;
                # the older SDK runners resolve paths from the product root.
                asset_root = config_path.parent if config.get("method") == "supermimic" else CONFIG_ROOT
                for key in ("policy_file", "trajectory_file_npz", "reference_file", "onnx_file"):
                    if key not in config:
                        continue
                    if entry["tag"] == "rl_celebration":
                        # The original celebration state is an unreachable placeholder.
                        self.assertEqual(config[key], "")
                        continue
                    with self.subTest(mode=name, tag=entry["tag"], key=key):
                        self.assertTrue(config[key], "active asset path is empty")
                        path = asset_root / config[key]
                        self.assertTrue(path.is_file(), path)
                        self.assertGreater(path.stat().st_size, 0)

    def test_combined_walking_profile_replaces_lb_a_and_matches_terrain_contract(self):
        combined_path = CONFIG_ROOT / COMBINED_WALKING_PROFILE / "default.yaml"
        terrain_path = CONFIG_ROOT / "rl_walking_leolab_terrain_example/default.yaml"
        combined = load_yaml(combined_path)
        terrain = load_yaml(terrain_path)
        self.assertEqual(self.tasks["walk_leo"]["key"], ["LB", "A"])
        self.assertEqual(self.tasks["walk_leo"]["runner"][0]["param_tag"],
                         "rl_walking_leolab_example")
        self.assertEqual(combined["num_observations"], 84)
        self.assertEqual(combined["num_actions"], 25)
        self.assertTrue(combined["policy_recurrent"])
        self.assertEqual(combined["policy_file"],
                         COMBINED_WALKING_PROFILE + "/policies/boxing_policy.mnn")
        self.assertEqual([i for i, (a, b) in enumerate(zip(
            combined["joint_damping"], terrain["joint_damping"])) if a != b], [4, 5, 10, 11])
        for index in (4, 5, 10, 11):
            self.assertEqual(combined["joint_damping"][index], 2.0)
        for key in combined:
            if key not in {"policy_file", "joint_damping"}:
                self.assertEqual(combined[key], terrain[key], key)

    def test_taunt_policy_does_not_depend_on_archived_standard_walking_directory(self):
        taunt = load_yaml(CONFIG_ROOT / "rl_walking_leolab_taunt/default.yaml")
        terrain = load_yaml(CONFIG_ROOT / "rl_walking_leolab_terrain_example/default.yaml")
        self.assertNotIn("rl_walking_leolab_example/", taunt["policy_file"])
        taunt_policy = CONFIG_ROOT / taunt["policy_file"]
        terrain_policy = CONFIG_ROOT / terrain["policy_file"]
        self.assertTrue(taunt_policy.is_file())
        self.assertTrue(terrain_policy.is_file())
        self.assertEqual(file_sha256(taunt_policy), file_sha256(terrain_policy))


class T800SuperMimicMigrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old = CONFIG_ROOT / "old"
        cls.mode = load_yaml(CONFIG_ROOT / "mode.yaml")
        cls.tasks = {task["motion"]: task for task in
                     load_yaml(CONFIG_ROOT / "task_motion/default.yaml")["tasks"]}

    def test_old_actions_are_preserved_byte_for_byte_and_inactive(self):
        manifest = json.loads((self.old / "manifest.json").read_text())
        self.assertEqual(manifest["schema_version"], 1)
        actual_files = set()
        for directory in ARCHIVED_DIRS:
            compatibility_path = CONFIG_ROOT / directory
            if compatibility_path.is_symlink():
                self.assertEqual(compatibility_path.readlink(), Path("old") / directory)
            else:
                self.assertFalse(compatibility_path.exists(), directory)
            self.assertTrue((self.old / directory).is_dir(), directory)
            actual_files.update(path.relative_to(self.old).as_posix()
                                for path in (self.old / directory).rglob("*") if path.is_file())
        for directory in TAUNT_BACKUP_PACKAGES:
            self.assertFalse((CONFIG_ROOT / directory).exists(), directory)
            archived = self.old / "嘲讽" / directory
            self.assertTrue(archived.is_dir(), directory)
            actual_files.update(path.relative_to(self.old).as_posix()
                                for path in archived.rglob("*") if path.is_file())
        self.assertEqual(set(manifest["files"]), actual_files)
        for relative_path, expected in manifest["files"].items():
            with self.subTest(asset=relative_path):
                self.assertEqual(file_sha256(self.old / relative_path), expected)
        self.assertTrue((self.old / "README.md").is_file())
        for mode, entries in self.mode["mode"].items():
            for entry in entries:
                with self.subTest(mode=mode, tag=entry["tag"]):
                    self.assertNotIn(entry["tag"], REPLACED_TAGS)
                    self.assertNotIn(entry["scope"].split("/", 1)[0], ARCHIVED_DIRS | {"old"})
        self.assertTrue({"dance", "victory", "ridicule", "celebration"}.isdisjoint(self.tasks))
        old_mode = load_yaml(self.old / "mode.before_supermimic.yaml.backup")
        for entries in old_mode["mode"].values():
            self.assertTrue(REPLACED_TAGS <= {entry["tag"] for entry in entries})

    def test_four_new_keys_are_reachable_from_every_former_entry_state(self):
        old_tasks = load_yaml(self.old / "task_motion.before_supermimic.yaml.backup")["tasks"]
        old_names = {"dance", "victory", "ridicule"}
        entry_states = {task["motion"] for task in old_tasks
                        if old_names.intersection(task.get("manual_transition", []))}
        self.assertEqual(entry_states, {"pd_stand", "pd_stand_x", "pd_stand_y", "walk",
                                       "walk_custom", "walk_leo", "walk_leo_terrain"})
        for motion, (tag, button, directory, _) in SUPER_MOTIONS.items():
            with self.subTest(motion=motion):
                task = self.tasks[motion]
                self.assertEqual(task["key"], ["RB", button])
                self.assertEqual(task["period"], 0.02)
                self.assertEqual(task["runner"], [
                    {"name": "rl_supermimic_runner", "enabled": True, "param_tag": tag}])
                self.assertEqual(task["auto_transition"], "walk_leo")
                self.assertIn("passive", task["manual_transition"])
                for source in entry_states:
                    self.assertIn(motion, self.tasks[source]["manual_transition"], source)
                for entries in self.mode["mode"].values():
                    scopes = {entry["tag"]: entry["scope"] for entry in entries}
                    self.assertEqual(scopes[tag], directory + "/default")
                collisions = [name for name, other in self.tasks.items() if other["key"] == task["key"]]
                self.assertEqual(collisions, [motion])

    def test_exports_keep_original_assets_with_zero_extra_hold(self):
        self.assertEqual(len(SUPER_EXPORTS), 5)
        for directory, default_hash in SUPER_EXPORTS.items():
            package = CONFIG_ROOT / directory
            with self.subTest(package=directory):
                # Older exports had a one-second hold; the refreshed taunt
                # already exports zero. Preserve each package's original manifest.
                deployed_default = (package / "default.yaml").read_bytes()
                self.assertEqual(load_yaml(package / "default.yaml")["hold_seconds"], 0.0)
                exported_hold = load_yaml(package / "deploy_config.yaml")["hold_seconds"]
                exported_default = deployed_default.replace(
                    b"\nhold_seconds: 0.0\n",
                    f"\nhold_seconds: {float(exported_hold)}\n".encode())
                exported_default_hash = hashlib.sha256(exported_default).hexdigest()
                self.assertEqual(exported_default_hash, default_hash)
                manifest = json.loads((package / "export_manifest.json").read_text())
                self.assertEqual(manifest["validation"]["input_shape"], [1, 1520])
                self.assertEqual(manifest["validation"]["output_shape"], [1, 25])
                for asset, expected in manifest["artifact_sha256"].items():
                    actual = (exported_default_hash if asset == "default.yaml"
                              else file_sha256(package / asset))
                    self.assertEqual(actual, expected, asset)

    def test_prone_replacement_preserves_controls_transitions_and_other_recoveries(self):
        old_tasks = {task["motion"]: task for task in
                     load_yaml(self.old / "task_motion.before_supermimic.yaml.backup")["tasks"]}
        expected = dict(old_tasks["prone_to_stance"])
        expected["runner"] = [
            {"name": "rl_supermimic_runner", "enabled": True, "param_tag": "rl_prone_to_stance"}]
        self.assertEqual(self.tasks["prone_to_stance"], expected)
        prone = load_yaml(CONFIG_ROOT / PRONE_PACKAGE / "default.yaml")
        self.assertEqual(prone["task"], "prone")
        old_entries = {name for name, task in old_tasks.items()
                       if "prone_to_stance" in task.get("manual_transition", [])}
        entries = {name for name, task in self.tasks.items()
                   if "prone_to_stance" in task.get("manual_transition", [])}
        self.assertEqual(entries, old_entries)
        for mode, entries in self.mode["mode"].items():
            with self.subTest(mode=mode):
                scopes = {entry["tag"]: entry["scope"] for entry in entries}
                self.assertEqual(scopes["rl_prone_to_stance"], PRONE_PACKAGE + "/default")
        for motion in ("supine_to_stance", "getup", "getup2", "stance_to_supine"):
            with self.subTest(motion=motion):
                self.assertEqual(self.tasks[motion], old_tasks[motion])

    def test_reference_shapes_and_policy_history_contract(self):
        for directory in SUPER_EXPORTS:
            package = CONFIG_ROOT / directory
            config = load_yaml(package / "default.yaml")
            with self.subTest(package=directory):
                self.assertEqual(config["method"], "supermimic")
                self.assertEqual(config["robot"], "t800")
                self.assertEqual(config["control_dt"], 0.02)
                self.assertEqual(config["physics_dt"], 0.002)
                self.assertEqual(config["observation_dim"], 25 + 25 + 3 + 3 + 25 + 6)
                self.assertEqual(config["goal_dim"], 650)
                self.assertEqual(config["history_length"], 10)
                self.assertEqual(config["input_dim"], config["goal_dim"] +
                                 config["history_length"] * config["observation_dim"])
                self.assertEqual(config["output_dim"], 25)
                self.assertEqual(config["history_order"], "oldest_to_newest_frame_major")
                raw_action_export = (config["action_target_formula"] ==
                                     "default_joint_pos + action_scale * raw_action")
                action_name = "raw_action" if raw_action_export else "action_output"
                previous_action = "previous_raw_action" if raw_action_export else "previous_executed_action"
                self.assertEqual(config["action_target_formula"],
                                 "default_joint_pos + action_scale * " + action_name)
                self.assertEqual(config["observation_layout"], [
                    "q-default_q:25", "joint_velocity:25", "body_angular_velocity:3",
                    "projected_gravity:3", previous_action + ":25", "relative_root_orientation_6d:6"])
                if not raw_action_export:
                    self.assertEqual(config["action_output_contract"],
                        "export wrapper returns executed_action(raw_action)=clamp(raw_action,-20,20)")
                with np.load(package / config["reference_file"], allow_pickle=False) as reference:
                    frames = reference["joint_pos"].shape[0]
                    self.assertGreaterEqual(frames, 2)
                    shapes = {"joint_pos": (frames, 25), "joint_vel": (frames, 25),
                              "root_pos": (frames, 3), "root_quat": (frames, 4),
                              "root_rotation": (frames, 3, 3), "root_lin_vel": (frames, 3),
                              "root_ang_vel": (frames, 3), "goal": (frames, 650)}
                    for name, shape in shapes.items():
                        values = reference[name]
                        self.assertEqual(values.shape, shape, name)
                        self.assertEqual(values.dtype, np.dtype("float32"), name)
                        self.assertTrue(np.isfinite(values).all(), name)
                    np.testing.assert_allclose(np.linalg.norm(reference["root_quat"], axis=1), 1.0, atol=2e-4)
                    rotations = reference["root_rotation"]
                    np.testing.assert_allclose(rotations.transpose(0, 2, 1) @ rotations,
                        np.broadcast_to(np.eye(3), (frames, 3, 3)), atol=2e-4)
                with np.load(package / "policies/parity_inputs.npz", allow_pickle=False) as parity:
                    self.assertEqual(parity["obs"].shape[1], 1520)
                    self.assertEqual(parity["actions"].shape, (parity["obs"].shape[0], 25))
                    self.assertTrue(np.isfinite(parity["obs"]).all())
                    self.assertTrue(np.isfinite(parity["actions"]).all())
                    if not raw_action_export:
                        self.assertLessEqual(float(np.abs(parity["actions"]).max()), 20.00001)

    def test_each_policy_joint_maps_to_one_sdk_joint_with_the_same_semantics(self):
        sdk_joints = [joint for limb in load_yaml(CONFIG_ROOT / "model/default.yaml")["limbs"]
                      for joint in limb["joints"]]
        self.assertEqual(len(sdk_joints), 25)
        self.assertEqual(len(set(sdk_joints)), 25)
        for directory in SUPER_EXPORTS:
            package = CONFIG_ROOT / directory
            config = load_yaml(package / "default.yaml")
            mapping = json.loads((package / "export_manifest.json").read_text())["joint_mapping"]
            with self.subTest(package=directory):
                self.assertEqual(config["mujoco_joint_names"], sdk_joints)
                self.assertEqual(len(config["joint_names"]), 25)
                self.assertEqual(len(set(config["joint_names"])), 25)
                self.assertEqual(set(mapping), set(config["joint_names"]))
                self.assertEqual(set(mapping.values()), set(sdk_joints))
                for policy_joint, sdk_joint in zip(config["joint_names"], sdk_joints):
                    self.assertEqual(mapping[policy_joint], sdk_joint)
                    # Numeric prefixes differ on the right arm and head. Match
                    # the complete joint role/side, rather than those prefixes.
                    self.assertEqual(policy_joint.split("_", 1)[1], sdk_joint.split("_", 1)[1])
                for name in ("default_joint_pos", "action_scale", "joint_stiffness", "joint_damping"):
                    values = np.asarray(config[name])
                    self.assertEqual(values.shape, (25,), name)
                    self.assertTrue(np.isfinite(values).all(), name)


if __name__ == "__main__":
    unittest.main()

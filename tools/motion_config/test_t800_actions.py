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
# The slim checkout has no historical asset archive. Active formal actions are
# checked against this deployment contract.
MAIN_WALKING_PROFILE = "rl_walking_leolab_impact_v2_dr_171000"
PRONE_PACKAGE = "rl_supermimic_prone_j12768_m22000"
RECOVERY_ENTRY_KEYS = {
    "entry_transition_enabled", "entry_transition_duration",
    "entry_transition_min_duration", "entry_transition_max_duration",
    "entry_transition_max_joint_velocity", "entry_transition_max_joint_acceleration",
    "entry_transition_reference_pose_weight", "entry_transition_source_tracking_error",
    "entry_transition_to_reference_pose", "entry_transition_reference_pose_tolerance",
    "entry_transition_reference_pose_settle_cycles",
    "entry_transition_reference_pose_arm_stiffness_scale",
}
PRONE_PLAYBACK_KEYS = {"skip_terminal_hold_frames"}

# These action IDs and physical bindings are the requested deployment contract.
DEPLOYED_ACTIONS = {
    6: (["RB", "CROSS_X_UP"], "rl_supermimic_local_clip_zq_xjh_3_10_02_strike_04"),
    9: (["RB", "CROSS_X_DOWN"], "rl_supermimic_local_clip_zq_xjh_3_5_01_strike_10"),
    11: (["RB", "CROSS_Y_LEFT"], "rl_supermimic_local_clip_zq_xjh_3_7_01_strike_09"),
    13: (["RB", "CROSS_Y_RIGHT"], "rl_supermimic_local_clip_zq_xjh_3_9_01_strike_05"),
    14: (["RB", "B"], "rl_supermimic_local_clip_zq_xjh_4_10_01_strike_02"),
    15: (["RB", "A"], "rl_supermimic_local_clip_zq_xjh_4_14_01_strike_04"),
    16: (["CROSS_X_UP", "CROSS_X_DOWN", "CROSS_Y_LEFT"], "rl_supermimic_local_clip_zq_xjh_4_1_01_strike_07"),
    17: (["CROSS_X_UP", "CROSS_X_DOWN", "CROSS_Y_RIGHT"], "rl_supermimic_local_clip_zq_xjh_4_2_01_strike_02"),
    19: (["RB", "Y"], "rl_supermimic_local_clip_zq_xjh_4_4_01_strike_01"),
    20: (["RB", "X"], "rl_supermimic_local_clip_zq_xjh_4_5_01_strike_06"),
}
DEPLOYED_MOTIONS = [f"supermimic_action_{number:02d}" for number in DEPLOYED_ACTIONS]
DEPLOYED_WALKS = {
    "walk_v2_dr_194500": ("LB", "CROSS_X_UP", "rl_walking_leolab_impact_v2_dr_194500"),
    "walk_v2_dr_191500": ("LB", "CROSS_Y_RIGHT", "rl_walking_leolab_impact_v2_dr_191500"),
    "walk_plus_origin_180000": ("LB", "CROSS_X_DOWN", "rl_walking_leolab_impact_v2_plus_origin_180000"),
    "walk_plus_dr_189000": ("LB", "CROSS_Y_LEFT", "rl_walking_leolab_impact_v2_plus_dr_189000"),
}
DEPLOYED_WALK_MOTIONS = list(DEPLOYED_WALKS)
NEW_TAGS = {"rl_" + motion for motion in DEPLOYED_MOTIONS} | {
    tag for _, _, tag in DEPLOYED_WALKS.values()
}
SUPER_PACKAGES = {"numbered_actions/" + package for _, package in DEPLOYED_ACTIONS.values()} | {PRONE_PACKAGE}

# The punch references have a short live-policy entry bridge.  Their low-speed
# reference tails are trimmed by the frame counts measured from the imported
# 50 Hz trajectories.  Leg references remain unmodified for later contact
# timing experiments.
ACTION_SKIP_TAILS = {6: 12, 9: 8, 11: 8, 13: 13, 14: 0, 15: 0, 16: 0, 17: 0, 19: 0, 20: 0}
SHORT_ENTRY_ACTIONS = {6, 9, 11, 13}
SHORT_ENTRY_VALUES = {
    "entry_transition_enabled": True,
    "entry_transition_duration": 0.12,
    "entry_transition_min_duration": 0.08,
    "entry_transition_max_duration": 0.16,
    "entry_transition_max_joint_velocity": 8.0,
    "entry_transition_max_joint_acceleration": 120.0,
    "entry_transition_reference_pose_weight": 0.0,
    "entry_transition_source_tracking_error": 0.75,
}

# Retained runtime bytes and parsed settings for active packages only.
RUNTIME_PACKAGES = {
    "rl_supermimic_prone_j12768_m22000": (
        "3d7191c888819682a6ee81a06247923bf8f872a1cfc1c27ee5c58ad92708fb6e", {
            "policies/policy.mnn": "29d8e4c4266f0fd22b38845a884a33f934af55a743837ab4d2fa6eb0409dbf7d",
            "trajectories/supermimic_reference.npz": "06de307bf052f33dafe2d1f3429fe769218d9730fcd961ecc91679fc606df9ed",
        }),
    "rl_supine_to_stance_v1_hybrid_9843_m13000": (
        "8be78069198b27c82e4f7cb569d0cd18aa78b3a5095eb7cfa39fd48ed5c2baf2", {
            "policies/policy.mnn": "3347630790f7ff69ef0dce1581d2f252d1d6afd00837a132820c6bbce8e8db7f",
            "trajectories/T800_supine_to_stance_sdk_clip_50hz.npz": "f46be05840d992d90762c16f09f585961e55b0a328123da88380089290cf0492",
        }),
    "rl_walking_leolab_impact_v2_dr_194500": (
        "1d6939da24eac9c835d6878f1511aecb75a5baa1d771791ddab0f83512134aee", {
            "policies/actor.onnx": "e2ea73ac9be201594e6bf8b934176abd61553fe31c9dab6e81b7663ff681732c",
            "policies/actor.yml": "30bb93f69daccfb93a7d85156ba44ca1c3f16fc4239222b9cc97946f8ace2ae0",
            "policies/boxing_policy.mnn": "bb1db4e38dc7731e23a701ee219e17c63a39f417a505748df994164956134549",
            "policies/deployment_contract.json": "4970538b90af2a4e651c3dba81d983450ee67381752ff1d88000843bacec5418",
        }),
    "rl_walking_leolab_impact_v2_dr_191500": (
        "b6163ec581b2b45f6276864658993f646cfc6c53200cb5f25152f8d98aa71a23", {
            "policies/actor.onnx": "9c6a3d53f85b4920908f71c4c90dda05c996de148bb4e7fbe7b7717de94ebf95",
            "policies/actor.yml": "30bb93f69daccfb93a7d85156ba44ca1c3f16fc4239222b9cc97946f8ace2ae0",
            "policies/boxing_policy.mnn": "bf4e86c2fe4642f1b347cc4c45947306451632541b1c07045f91ac152e6172f8",
            "policies/deployment_contract.json": "38028dfc53c2d4b1b0f3dada4c1e4689f6b67e8a8ed8ac18a2e5171ec4fb8e91",
        }),
    "rl_walking_leolab_impact_v2_plus_origin_180000": (
        "3c643accdf694de52e9b9ad9ee56acf814015c7b7d5a02e4123071906bfd09c0", {
            "policies/actor.onnx": "936fbded06ef58eed3e6d2ddf9a4fe4f28e6408a14382be2e933fd9130d1a108",
            "policies/actor.yml": "30bb93f69daccfb93a7d85156ba44ca1c3f16fc4239222b9cc97946f8ace2ae0",
            "policies/boxing_policy.mnn": "9b650cf2166bfe0c0721ff0c21fbb337a18aa8ca4fe6403b184bbf4aecd49ec5",
            "policies/deployment_contract.json": "aa24247502594406ac09d7432af0cfdaceb28479223086bcfe4c8c53d031672c",
        }),
    "rl_walking_leolab_impact_v2_plus_dr_189000": (
        "e0c2500c5bf9c9862541169eb5f5e25c84bbd675975425390917611946bd7341", {
            "policies/actor.onnx": "6bac07f1a7b8fafe7b879605b5a57ca8f504d709170c45c74991559a832c5a46",
            "policies/actor.yml": "30bb93f69daccfb93a7d85156ba44ca1c3f16fc4239222b9cc97946f8ace2ae0",
            "policies/boxing_policy.mnn": "d511ab6564f2d6157e45ec68f5503d7dadfd39dd756db27d99f0cc8a94a1ff84",
            "policies/deployment_contract.json": "518fa4004c626a8d57c694cd089a0d7bb00bf0bf3af915026e9a87b7034560c3",
        }),
    "rl_walking_leolab_impact_v2_dr_171000": (
        "5153ed5c751d525d46c566e7d3c6f479d9f6a9e105b3986095451b02e59a3ccf", {
            "policies/actor.onnx": "12dc961ec09e3877b32d68059abb99f256b37d46f2d251a6beb522b3ff7c39b2",
            "policies/actor.yml": "30bb93f69daccfb93a7d85156ba44ca1c3f16fc4239222b9cc97946f8ace2ae0",
            "policies/boxing_policy.mnn": "0ee66af75502714110f5fe02f603ec8be36f22eea3472d3cd1cf535e9fe81080",
            "policies/deployment_contract.json": "d863db0c6e0a07ddce570b6949a9e6dafdfc4c93ffc3300c7194466e97cb4faa",
        }),
}


# These existing motions retain their keys, runners and periods after migration.
PRESERVED_MOTIONS = {
    "walk": (0.01, ["LB", "B"], "rl_walking_example_runner", "rl_walking_example"),
    "walk_leo": (0.02, ["LB", "A"], "rl_walking_leolab_example_runner", "rl_walking_leolab_example"),
    "leo_taunt": (0.02, ["LB", "Y"],
                  "rl_walking_leolab_example_runner", "rl_walking_leolab_taunt"),
    "supine_to_stance": (0.02, ["START", "CROSS_X_UP"], "rl_dance_example_runner", "rl_supine_to_stance"),
    "getup": (0.02, ["START", "CROSS_Y_LEFT"], "rl_getup_example_runner", "rl_getup_example"),
    "getup2": (0.02, ["LB", "START"], "rl_getup_example_runner", "rl_getup2_example"),
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
    "rl_walking_leolab_example": MAIN_WALKING_PROFILE + "/default",
    "rl_walking_leolab_taunt": "rl_walking_leolab_taunt/default",
    "rl_getup_example": "rl_getup_example/default",
    "rl_getup2_example": "rl_getup2_example/default",
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
            stable = {tag: scope for tag, scope in scopes.items() if tag not in NEW_TAGS}
            self.assertEqual(stable, expected)
            for tag, scope in scopes.items():
                with self.subTest(mode=name, tag=tag):
                    self.assertTrue((CONFIG_ROOT / (scope + ".yaml")).is_file(), scope)
            for task in self.task_list:
                for runner in task.get("runner", []):
                    if runner.get("enabled") and "param_tag" in runner:
                        with self.subTest(mode=name, motion=task["motion"]):
                            self.assertIn(runner["param_tag"], scopes)

    def test_other_motions_keep_keys_runners_tags_and_periods(self):
        self.assertEqual(len(PRESERVED_MOTIONS), 7)
        walking = ["walk", "walk_leo", *DEPLOYED_WALK_MOTIONS]
        returns = ["passive", "pd_stand"] + walking
        transitions = {
            "walk": ([
                "passive", "pd_stand", "walk_leo", "getup", "getup2",
                "stance_to_supine", *DEPLOYED_MOTIONS, *DEPLOYED_WALK_MOTIONS,
            ], None),
            "walk_leo": ([
                "passive", "pd_stand", "walk", "leo_taunt", "getup",
                "getup2", "stance_to_supine", *DEPLOYED_MOTIONS, *DEPLOYED_WALK_MOTIONS,
            ], None),
        }
        for name in DEPLOYED_WALK_MOTIONS:
            transitions[name] = ([
                "passive", "pd_stand", "walk", "walk_leo",
                *[other for other in DEPLOYED_WALK_MOTIONS if other != name],
                "getup", "getup2", "stance_to_supine", *DEPLOYED_MOTIONS,
            ], None)
        transitions.update({name: (returns, "walk_leo") for name in ["getup", "getup2"]})
        transitions.update({
            "leo_taunt": (["walk_leo"], "walk_leo"),
            "supine_to_stance": (["passive", "pd_stand", "walk_leo"], "walk_leo"),
            "stance_to_supine": (["passive", "pd_stand"], "passive"),
        })
        for name, expected in PRESERVED_MOTIONS.items():
            with self.subTest(motion=name):
                task = self.tasks[name]
                period, key, runner, tag = expected
                self.assertEqual(task["period"], period)
                self.assertEqual(task["key"], key)
                self.assertEqual(task["runner"], [{"name": runner, "enabled": True, "param_tag": tag}])
                self.assertEqual((task["manual_transition"], task.get("auto_transition")), transitions[name])

    def test_new_walking_motions_use_dedicated_profiles_and_keys(self):
        for name, (button, direction, tag) in DEPLOYED_WALKS.items():
            with self.subTest(motion=name):
                task = self.tasks[name]
                self.assertEqual(task["key"], [button, direction])
                self.assertEqual(task["period"], 0.02)
                self.assertEqual(task["runner"], [{
                    "name": "rl_walking_leolab_example_runner",
                    "enabled": True,
                    "param_tag": tag,
                }])
                config = load_yaml(CONFIG_ROOT / tag / "default.yaml")
                self.assertEqual(config["policy_file"], tag + "/policies/boxing_policy.mnn")
                self.assertEqual(config["num_observations"], 84)
                self.assertEqual(config["num_actions"], 25)
                self.assertTrue(config["policy_recurrent"])

    def test_active_policy_and_trajectory_paths_exist(self):
        for name, entries in self.mode["mode"].items():
            for entry in entries:
                config_path = CONFIG_ROOT / (entry["scope"] + ".yaml")
                config = load_yaml(config_path)
                # SuperMimic exports keep their package-relative asset paths;
                # the older SDK runners resolve paths from the product root.
                asset_root = config_path.parent if config.get("method") == "supermimic" else CONFIG_ROOT
                for key in ("policy_file", "trajectory_file_npz", "reference_file"):
                    if key not in config:
                        continue
                    with self.subTest(mode=name, tag=entry["tag"], key=key):
                        self.assertTrue(config[key], "active asset path is empty")
                        path = asset_root / config[key]
                        self.assertTrue(path.is_file(), path)
                        self.assertGreater(path.stat().st_size, 0)

    def test_main_walking_profile_uses_former_dpad_right_model(self):
        combined_path = CONFIG_ROOT / MAIN_WALKING_PROFILE / "default.yaml"
        combined = load_yaml(combined_path)
        self.assertEqual(self.tasks["walk_leo"]["key"], ["LB", "A"])
        self.assertEqual(self.tasks["walk_leo"]["runner"][0]["param_tag"],
                         "rl_walking_leolab_example")
        self.assertEqual(combined["num_observations"], 84)
        self.assertEqual(combined["num_actions"], 25)
        self.assertTrue(combined["policy_recurrent"])
        self.assertEqual(combined["policy_file"],
                         MAIN_WALKING_PROFILE + "/policies/boxing_policy.mnn")
        for index in (4, 5, 10, 11):
            self.assertEqual(combined["joint_damping"][index], 2.0)

    def test_taunt_policy_uses_the_active_walking_directory(self):
        taunt = load_yaml(CONFIG_ROOT / "rl_walking_leolab_taunt/default.yaml")
        combined = load_yaml(CONFIG_ROOT / MAIN_WALKING_PROFILE / "default.yaml")
        self.assertEqual(taunt["policy_file"], combined["policy_file"])
        taunt_policy = CONFIG_ROOT / taunt["policy_file"]
        combined_policy = CONFIG_ROOT / combined["policy_file"]
        self.assertTrue(taunt_policy.is_file())
        self.assertTrue(combined_policy.is_file())
        self.assertEqual(file_sha256(taunt_policy), file_sha256(combined_policy))

    def test_removed_walking_profiles_and_keys_are_absent(self):
        self.assertNotIn("walk_impact", self.tasks)
        self.assertNotIn("walk_v2_dr_171000", self.tasks)
        self.assertNotIn("walk_plus_194500", self.tasks)
        self.assertFalse((CONFIG_ROOT / "rl_walking_leolab_ankle_combined_v2_140000").exists())
        self.assertFalse((CONFIG_ROOT / "rl_walking_leolab_impact_v2_154500").exists())
        self.assertFalse((CONFIG_ROOT / "rl_walking_leolab_impact_v2_plus_194500").exists())
        tags = {entry["tag"] for entries in self.mode["mode"].values() for entry in entries}
        self.assertNotIn("rl_walking_leolab_impact_v2", tags)


class T800SuperMimicMigrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old = CONFIG_ROOT / "old"
        cls.mode = load_yaml(CONFIG_ROOT / "mode.yaml")
        cls.tasks = {task["motion"]: task for task in
                     load_yaml(CONFIG_ROOT / "task_motion/default.yaml")["tasks"]}

    def test_old_actions_keep_only_runtime_files_and_remain_inactive(self):
        self.assertFalse(self.old.exists())
        for mode, entries in self.mode["mode"].items():
            for entry in entries:
                with self.subTest(mode=mode, tag=entry["tag"]):
                    self.assertFalse(entry["scope"].startswith("old/"))
        self.assertTrue({"dance", "victory", "ridicule", "celebration"}.isdisjoint(self.tasks))

    def test_ten_formal_actions_keep_runtime_assets_without_a_test_catalog(self):
        self.assertFalse((CONFIG_ROOT / "numbered_actions/catalog.json").exists())
        self.assertEqual({task for task in self.tasks if task.startswith("supermimic_")},
                         set(DEPLOYED_MOTIONS))
        ready = {"pd_stand", "pd_stand_x", "pd_stand_y", "walk",
                 "walk_leo", *DEPLOYED_WALK_MOTIONS}
        for number, (key, package) in DEPLOYED_ACTIONS.items():
            motion = f"supermimic_action_{number:02d}"
            with self.subTest(motion=motion, number=number):
                task = self.tasks[motion]
                tag = "rl_" + motion
                self.assertEqual(task["key"], key)
                self.assertEqual(task["period"], 0.02)
                self.assertEqual(task["runner"], [{"name": "rl_supermimic_runner",
                                 "enabled": True, "param_tag": tag}])
                self.assertEqual(task["auto_transition"], "walk_leo")
                self.assertEqual(task["manual_transition"], ["passive", "pd_stand", "walk",
                                 "walk_leo"])
                for entries in self.mode["mode"].values():
                    scopes = {entry["tag"]: entry["scope"] for entry in entries}
                    self.assertEqual(scopes[tag], "numbered_actions/" + package + "/default")
                entries = {source for source, state in self.tasks.items()
                           if motion in state.get("manual_transition", [])}
                self.assertEqual(entries, ready)
                config = load_yaml(CONFIG_ROOT / "numbered_actions" / package / "default.yaml")
                self.assertEqual(config["hold_seconds"], 0.0)
                self.assertEqual(config.get("skip_terminal_hold_frames", 0), ACTION_SKIP_TAILS[number])
                self.assertFalse(config.get("entry_transition_to_reference_pose", False))
                if number in SHORT_ENTRY_ACTIONS:
                    for field, value in SHORT_ENTRY_VALUES.items():
                        with self.subTest(motion=motion, field=field):
                            self.assertEqual(config[field], value)
                else:
                    self.assertNotIn("entry_transition_duration", config)

    def test_physical_key_masks_are_unique_and_unused_combinations_stay_empty(self):
        names = ("LB", "RB", "A", "B", "X", "Y", "BACK", "START",
                 "CROSS_X_UP", "CROSS_X_DOWN", "CROSS_Y_LEFT", "CROSS_Y_RIGHT")
        masks = {}
        for name, task in self.tasks.items():
            with self.subTest(motion=name):
                self.assertTrue(task["key"])
                self.assertEqual(len(task["key"]), len(set(task["key"])))
                mask = sum(1 << names.index(key) for key in set(task["key"]))
                self.assertNotIn(mask, masks, f"{name} collides with {masks.get(mask)}")
                masks[mask] = name
        # RT is the vertical marker pair, so RT+up/down cannot be distinct.
        self.assertNotIn((1 << 8) | (1 << 9), masks)

    def test_historical_archive_is_omitted(self):
        self.assertFalse(self.old.exists())

    def test_previous_competition_packages_are_omitted(self):
        self.assertFalse(self.old.exists())

    def test_runtime_packages_preserve_settings_and_asset_bytes(self):
        for directory, (expected_config, assets) in RUNTIME_PACKAGES.items():
            package = CONFIG_ROOT / directory
            with self.subTest(package=directory):
                self.assertEqual({path.relative_to(package).as_posix()
                                  for path in package.rglob("*") if path.is_file()},
                                 {"default.yaml"} | set(assets))
                config = load_yaml(package / "default.yaml")
                # Prone preparation now matches supine; the original export,
                # policy gains, reference and playback settings stay intact.
                excluded = RECOVERY_ENTRY_KEYS | (PRONE_PLAYBACK_KEYS if directory == PRONE_PACKAGE else set())
                preserved_config = ({key: value for key, value in config.items()
                                     if key not in excluded}
                                    if directory == PRONE_PACKAGE else config)
                serialized = json.dumps(preserved_config, sort_keys=True, separators=(",", ":"))
                self.assertEqual(hashlib.sha256(serialized.encode()).hexdigest(), expected_config)
                for name, expected in assets.items():
                    self.assertEqual(file_sha256(package / name), expected, name)
                asset_root = package if config.get("method") == "supermimic" else CONFIG_ROOT
                for key in ("policy_file", "reference_file", "trajectory_file_npz"):
                    if key in config:
                        self.assertTrue((asset_root / config[key]).is_file(), config[key])
                if directory in SUPER_PACKAGES:
                    self.assertEqual(config["hold_seconds"], 0.0)

    def test_prone_and_supine_share_pd_preparation_and_settle_conditions(self):
        prone = load_yaml(CONFIG_ROOT / PRONE_PACKAGE / "default.yaml")
        supine = load_yaml(CONFIG_ROOT / "rl_supine_to_stance_v1_hybrid_9843_m13000/default.yaml")
        for key in RECOVERY_ENTRY_KEYS:
            with self.subTest(parameter=key):
                self.assertIn(key, prone)
                self.assertEqual(prone[key], supine[key])
        self.assertTrue(prone["entry_transition_to_reference_pose"])
        self.assertEqual(prone["entry_transition_reference_pose_tolerance"], 0.80)
        self.assertEqual(prone["entry_transition_reference_pose_settle_cycles"], 5)
        self.assertEqual(prone["control_dt"] *
                         prone["entry_transition_reference_pose_settle_cycles"], 0.10)
        self.assertEqual(prone["hold_seconds"], 0)
        self.assertEqual(prone["skip_terminal_hold_frames"], 12)

    def test_formal_strikes_keep_live_policy_entry(self):
        for directory in SUPER_PACKAGES - {PRONE_PACKAGE}:
            with self.subTest(package=directory):
                config = load_yaml(CONFIG_ROOT / directory / "default.yaml")
                self.assertFalse(config.get("entry_transition_to_reference_pose", False))

    def test_prone_replacement_preserves_controls_transitions_and_other_recoveries(self):
        self.assertEqual(self.tasks["prone_to_stance"], {
            "motion": "prone_to_stance", "period": 0.02,
            "runner": [{"name": "rl_supermimic_runner", "enabled": True,
                        "param_tag": "rl_prone_to_stance"}],
            "manual_transition": ["passive", "pd_stand", "walk_leo"],
            "auto_transition": "walk_leo", "key": ["START", "CROSS_Y_RIGHT"],
        })
        prone = load_yaml(CONFIG_ROOT / PRONE_PACKAGE / "default.yaml")
        self.assertEqual(prone["task"], "prone")
        entries = {name for name, task in self.tasks.items()
                   if "prone_to_stance" in task.get("manual_transition", [])}
        self.assertEqual(entries, {"passive", "pd_stand", "pd_stand_x", "pd_stand_y"})
        for mode, entries in self.mode["mode"].items():
            with self.subTest(mode=mode):
                scopes = {entry["tag"]: entry["scope"] for entry in entries}
                self.assertEqual(scopes["rl_prone_to_stance"], PRONE_PACKAGE + "/default")
        recovery_transitions = {
            "supine_to_stance": (["passive", "pd_stand", "walk_leo"], "walk_leo"),
            "getup": (["passive", "pd_stand", "walk", "walk_leo",
                       *DEPLOYED_WALK_MOTIONS], "walk_leo"),
            "getup2": (["passive", "pd_stand", "walk", "walk_leo",
                        *DEPLOYED_WALK_MOTIONS], "walk_leo"),
            "stance_to_supine": (["passive", "pd_stand"], "passive"),
        }
        for motion, (manual, automatic) in recovery_transitions.items():
            with self.subTest(motion=motion):
                period, key, runner, tag = PRESERVED_MOTIONS[motion]
                self.assertEqual(self.tasks[motion], {
                    "motion": motion, "period": period, "key": key,
                    "runner": [{"name": runner, "enabled": True, "param_tag": tag}],
                    "manual_transition": manual, "auto_transition": automatic,
                })

    def test_reference_shapes_and_policy_history_contract(self):
        for directory in SUPER_PACKAGES:
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
                output_contract = config.get("action_output_contract")
                # The local-clip exports call their target variable raw_action,
                # but explicitly declare that the MNN wrapper returns executed
                # (clamped) actions. History follows that exported output.
                previous_action = ("previous_raw_action" if raw_action_export and not output_contract
                                   else "previous_executed_action")
                self.assertEqual(config["action_target_formula"],
                                 "default_joint_pos + action_scale * " + action_name)
                self.assertEqual(config["observation_layout"], [
                    "q-default_q:25", "joint_velocity:25", "body_angular_velocity:3",
                    "projected_gravity:3", previous_action + ":25", "relative_root_orientation_6d:6"])
                if not raw_action_export or output_contract:
                    self.assertIn(output_contract, (
                        "export wrapper returns executed_action(raw_action)=clamp(raw_action,-20,20)",
                        "clamp(raw_action,-20,20); previous action is executed action",
                        "executed_action(raw_action)=clamp(raw_action,-20,20)"))
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

    def test_each_policy_joint_maps_to_one_sdk_joint_with_the_same_semantics(self):
        sdk_joints = [joint for limb in load_yaml(CONFIG_ROOT / "model/default.yaml")["limbs"]
                      for joint in limb["joints"]]
        self.assertEqual(len(sdk_joints), 25)
        self.assertEqual(len(set(sdk_joints)), 25)
        for directory in SUPER_PACKAGES:
            package = CONFIG_ROOT / directory
            config = load_yaml(package / "default.yaml")
            with self.subTest(package=directory):
                self.assertEqual(config["mujoco_joint_names"], sdk_joints)
                self.assertEqual(len(config["joint_names"]), 25)
                self.assertEqual(len(set(config["joint_names"])), 25)
                for policy_joint, sdk_joint in zip(config["joint_names"], sdk_joints):
                    # Numeric prefixes differ on the right arm and head. Match
                    # the complete joint role/side, rather than those prefixes.
                    self.assertEqual(policy_joint.split("_", 1)[1], sdk_joint.split("_", 1)[1])
                for name in ("default_joint_pos", "action_scale", "joint_stiffness", "joint_damping"):
                    values = np.asarray(config[name])
                    self.assertEqual(values.shape, (25,), name)
                    self.assertTrue(np.isfinite(values).all(), name)


if __name__ == "__main__":
    unittest.main()

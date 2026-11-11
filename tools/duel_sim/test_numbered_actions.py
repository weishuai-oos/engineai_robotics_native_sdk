"""Numbered action registration without SDK binaries, Qt, or a robot."""

import contextlib
import io
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from tools.duel_sim import launcher, numbered_actions, single_launcher
from tools.virtual_gamepad.action_catalog import (
    BUTTON_NAMES, READY_MOTIONS, action_keys, motion_name, task_keys,
)


class NumberedActionsTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.repo = self.root / "repo"
        self.assets = self.repo / "assets"
        self.config = self.assets / "config/t800"
        self.config.mkdir(parents=True)
        source = launcher.REPO_ROOT / "assets/config/t800"
        self.mode = (source / "mode.yaml").read_text()
        self.tasks = (source / "task_motion/default.yaml").read_text()
        (self.config / "mode.yaml").write_text(self.mode)
        for relative, text in {
            "task_motion/default.yaml": self.tasks,
            "pd_stand/default.yaml": "desired_joint_position:\n  - [0.4, -0.8]\nstiffness: 1080\n",
            "lcm/default.yaml": "multicast: false\nip_port: 239.255.76.67:7667\nttl: 0\n",
            "ros2_bridge/default.yaml": "topics:\n  state: /hardware/joint_state\n",
            "google_logger/default.yaml": "glog_dir: ./logs\n",
        }.items():
            path = self.config / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
        self.actions = [
            {"id": 1, "label": "左拳", "batch": 1, "iteration": 120000,
             "package": "rl_supermimic_s2_left_j123_m120000"},
            {"id": 27, "label": "右拳", "batch": 5, "iteration": 52000,
             "package": "rl_supermimic_s2_right_j456_m52000"},
        ]
        self.catalog = self.config / "numbered_actions/catalog.json"
        self.write_catalog(self.actions)
        for action in self.actions:
            for relative in ("default.yaml", "policies/policy.mnn",
                             "trajectories/supermimic_reference.npz"):
                path = self.catalog.parent / action["package"] / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b"runtime test asset")

    def write_catalog(self, actions):
        self.catalog.parent.mkdir(parents=True, exist_ok=True)
        self.catalog.write_text(json.dumps({"schema_version": 1, "capacity": 50,
                                            "actions": actions}))

    def snapshot(self):
        return {path.relative_to(self.assets): path.read_bytes()
                for path in self.assets.rglob("*") if path.is_file()}

    def test_all_fifty_commands_have_unique_keys_and_match_virtual_encoding(self):
        values = []
        keys = [*BUTTON_NAMES, "BACK", "START", "CROSS_X_UP", "CROSS_X_DOWN",
                "CROSS_Y_LEFT", "CROSS_Y_RIGHT"]
        for number in range(1, 51):
            value = sum(1 << keys.index(key) for key in task_keys(number))
            virtual = action_keys(number)
            self.assertEqual(virtual[:2], ("LT", "RT"))
            encoded = 0xF00 | sum(1 << keys.index(key) for key in virtual[2:])
            self.assertEqual(value, encoded)
            values.append(value)
        self.assertEqual(len(set(values)), 50)
        original_keys = re.findall(r"(?m)^    key: \[([^\]]+)\]", self.tasks)
        original_values = {sum(1 << keys.index(key.strip()) for key in row.split(","))
                           for row in original_keys}
        self.assertTrue(set(values).isdisjoint(original_values))

    def test_local_numbered_policies_skip_static_tail(self):
        root = launcher.REPO_ROOT / "assets/config/t800"
        catalog_path = root / "numbered_actions/catalog.json"
        if not catalog_path.is_file():
            self.skipTest("No personal numbered policy catalog installed")
        catalog = json.loads(catalog_path.read_text())
        for action in catalog["actions"]:
            with self.subTest(number=action["id"]):
                text = (root / "numbered_actions" / action["package"] / "default.yaml").read_text()
                hold = re.search(r"(?m)^hold_seconds:[ \t]*([^\n]+)", text)
                skip = re.search(r"(?m)^skip_terminal_hold_frames:[ \t]*(\d+)[ \t]*$", text)
                tail = re.search(r"(?m)^terminal_hold_frames:[ \t]*(\d+)[ \t]*$", text)
                self.assertIsNotNone(hold)
                self.assertEqual(float(hold[1]), 0)
                self.assertIsNotNone(skip)
                self.assertEqual(int(skip[1]), int(tail[1]) if tail else 0)

    def test_registration_preserves_every_original_task_and_hardware_mode(self):
        before = self.snapshot()
        assets = launcher.copy_assets_config(self.assets, self.root / "overlay")
        config = assets / "config/t800"
        catalog = numbered_actions.configure_numbered_actions(config)
        self.assertEqual(catalog, config / "numbered_actions/catalog.json")
        graph = (config / "task_motion/numbered_actions.yaml").read_text()
        original_part = graph.split("\n  - motion: test_action_", 1)[0]
        for action in self.actions:
            original_part = original_part.replace(f"      - {motion_name(action['id'])}\n", "")
        self.assertEqual(original_part, self.tasks)
        for action in self.actions:
            name = motion_name(action["id"])
            self.assertEqual(graph.count(f"  - motion: {name}\n"), 1)
            self.assertEqual(graph.count(f"      - {name}\n"), len(READY_MOTIONS))
            block = graph.split(f"  - motion: {name}\n", 1)[1].split("\n  - motion:", 1)[0]
            self.assertIn(f"        param_tag: {name}\n", block)
            self.assertIn("    auto_transition: walk_leo\n", block)
            self.assertNotIn("      - test_action_", block)
        self.assertEqual((config / "task_motion/default.yaml").read_text(), self.tasks)
        mode = (config / "mode.yaml").read_text()
        self.assertEqual(mode.split("  sim:")[0], self.mode.split("  sim:")[0])
        self.assertIn("scope: task_motion/numbered_actions", mode)
        for action in self.actions:
            package = config / "numbered_actions" / action["package"]
            self.assertFalse((package / "default.yaml").is_symlink())
            self.assertTrue((package / "policies/policy.mnn").is_symlink())
        self.assertEqual(self.snapshot(), before)

    def test_single_uses_private_assets_and_preserves_pd_lcm_and_ros(self):
        before = self.snapshot()
        args = single_launcher.parse_args(["--repo", str(self.repo), "--virtual-gamepad"])
        plans = single_launcher.build_session(args, self.root / "logs", self.root / "runtime")
        sim, robot, virtual = plans
        assets = Path(robot["env"]["ENGINEAI_ROBOTICS_ASSETS"])
        self.assertEqual(assets, self.root / "runtime/assets")
        self.assertEqual(sim["env"]["ENGINEAI_ROBOTICS_ASSETS"], str(assets))
        self.assertEqual(robot["env"]["ENGINEAI_INPUT_SOURCE"], "virtual")
        for relative in ("pd_stand/default.yaml", "lcm/default.yaml", "ros2_bridge/default.yaml"):
            self.assertEqual((assets / "config/t800" / relative).read_bytes(),
                             (self.config / relative).read_bytes())
        self.assertIn("--test-catalog", virtual["command"])
        self.assertEqual(virtual["command"][-3:],
                         ["--lcm-url", "udpm://239.255.76.67:7667?ttl=0", "--connect"])
        self.assertEqual(before, self.snapshot())

    def test_duel_registers_only_virtual_robot_a(self):
        args = launcher.parse_args(["--repo", str(self.repo), "--virtual-gamepad"])
        with mock.patch.object(launcher, "generate_scene"):
            sim, first, second, virtual = launcher.build_session(
                args, self.root / "runtime", self.root / "logs")
        first_config = Path(first["env"]["ENGINEAI_ROBOTICS_CONFIG"]) / "t800"
        second_config = Path(second["env"]["ENGINEAI_ROBOTICS_CONFIG"]) / "t800"
        self.assertTrue((first_config / "task_motion/numbered_actions.yaml").is_file())
        self.assertFalse((second_config / "task_motion/numbered_actions.yaml").exists())
        self.assertNotIn("test_action_", (second_config / "mode.yaml").read_text())
        self.assertEqual(first["env"]["ENGINEAI_INPUT_SOURCE"], "virtual")
        self.assertEqual(second["env"]["ENGINEAI_INPUT_SOURCE"], "gamepad")
        self.assertIn(str(first_config / "numbered_actions/catalog.json"), virtual["command"])

    def test_catalog_does_not_enable_numbered_actions_for_physical_single(self):
        args = single_launcher.parse_args(["--repo", str(self.repo)])
        _, robot = single_launcher.build_session(args, self.root / "logs")
        self.assertEqual(robot["env"]["ENGINEAI_ROBOTICS_ASSETS"], str(self.assets))
        self.assertEqual((self.config / "mode.yaml").read_text(), self.mode)
        self.assertFalse((self.config / "task_motion/numbered_actions.yaml").exists())

    def test_missing_asset_rejected_before_any_config_write(self):
        (self.catalog.parent / self.actions[0]["package"] / "policies/policy.mnn").unlink()
        before = self.snapshot()
        with self.assertRaisesRegex(ValueError, "Missing numbered action asset"):
            numbered_actions.configure_numbered_actions(self.config)
        self.assertEqual(before, self.snapshot())

    def test_duplicate_and_out_of_range_catalogs_rejected(self):
        for actions in ([self.actions[0], self.actions[0]],
                        [dict(self.actions[0], id=51)], [dict(self.actions[0], id=True)]):
            with self.subTest(actions=actions):
                self.write_catalog(actions)
                before = self.snapshot()
                with self.assertRaises(ValueError):
                    numbered_actions.configure_numbered_actions(self.config)
                self.assertEqual(before, self.snapshot())

    def test_missing_or_empty_catalog_keeps_single_and_dual_virtual_controls(self):
        shutil.rmtree(self.catalog.parent)
        for variant in ("missing", "empty"):
            with self.subTest(catalog=variant):
                if variant == "empty":
                    self.write_catalog([])
                before = self.snapshot()
                self.assertIsNone(numbered_actions.configure_numbered_actions(self.config))
                single_args = single_launcher.parse_args(
                    ["--repo", str(self.repo), "--virtual-gamepad"])
                single = single_launcher.build_session(
                    single_args, self.root / "logs", self.root / variant / "single")
                dual_args = launcher.parse_args(
                    ["--repo", str(self.repo), "--virtual-gamepads"])
                with mock.patch.object(launcher, "generate_scene"):
                    dual = launcher.build_session(
                        dual_args, self.root / variant / "dual", self.root / "logs")
                self.assertNotIn("--test-catalog", single[-1]["command"])
                for virtual in dual[-2:]:
                    self.assertNotIn("--test-catalog", virtual["command"])
                for robot in (single[1], dual[1], dual[2]):
                    config = Path(robot["env"]["ENGINEAI_ROBOTICS_CONFIG"]) / "t800"
                    self.assertEqual(robot["env"]["ENGINEAI_INPUT_SOURCE"], "virtual")
                    self.assertNotIn("test_action_", (config / "mode.yaml").read_text())
                    self.assertFalse((config / "task_motion/numbered_actions.yaml").exists())
                self.assertEqual(before, self.snapshot())

    def test_ambiguous_config_rejected_before_writes(self):
        for text in (self.mode.replace("  sim:", "  sim:\n  sim:"),
                     self.mode.replace("scope: task_motion/default", "scope: custom/task")):
            (self.config / "mode.yaml").write_text(text)
            before = self.snapshot()
            with self.assertRaises(ValueError):
                numbered_actions.configure_numbered_actions(self.config)
            self.assertEqual(before, self.snapshot())

    def test_missing_ready_motion_rejected_before_any_config_write(self):
        (self.config / "task_motion/default.yaml").write_text(
            self.tasks.replace("  - motion: walk_leo\n", "  - motion: other_walk\n"))
        before = self.snapshot()
        with self.assertRaisesRegex(ValueError, "unique existing PD and walking tasks"):
            numbered_actions.configure_numbered_actions(self.config)
        self.assertEqual(before, self.snapshot())

    def test_linked_task_directory_cannot_write_back_to_original_assets(self):
        task_dir = self.config / "task_motion"
        target = self.root / "original_tasks"
        task_dir.rename(target)
        task_dir.symlink_to(target, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "private regular"):
            numbered_actions.configure_numbered_actions(self.config)
        self.assertEqual((target / "default.yaml").read_text(), self.tasks)
        self.assertFalse((target / "numbered_actions.yaml").exists())
        self.assertEqual((self.config / "mode.yaml").read_text(), self.mode)

    def test_no_catalog_preserves_legacy_registration(self):
        self.catalog.unlink()
        before = self.snapshot()
        self.assertIsNone(numbered_actions.configure_numbered_actions(self.config))
        self.assertEqual(before, self.snapshot())

    def test_single_runtime_cleaned_on_success_error_and_dry_run(self):
        for error, dry_run in ((False, False), (True, False), (False, True)):
            with self.subTest(error=error, dry_run=dry_run):
                observed = []
                real_build = single_launcher.build_session

                def build(args, logs, runtime=None):
                    observed.append(runtime)
                    self.assertTrue(runtime.is_dir())
                    return real_build(args, logs, runtime)

                options = ["--repo", str(self.repo), "--virtual-gamepad",
                           "--simulator", sys.executable, "--executor", sys.executable,
                           "--log-dir", str(self.root / "logs")]
                if dry_run:
                    options.append("--dry-run")
                with mock.patch.object(single_launcher, "existing_sdk_processes", return_value=[]), \
                        mock.patch.object(single_launcher, "build_session", side_effect=build), \
                        mock.patch.object(single_launcher, "run_session", side_effect=OSError("test failure") if error else None,
                                          return_value=0), \
                        contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(single_launcher.main(options), 1 if error else 0)
                self.assertEqual(len(observed), 1)
                self.assertFalse(observed[0].exists())

    def test_duel_runtime_cleaned_on_success_and_error(self):
        real_mkdtemp = tempfile.mkdtemp
        for error in (False, True):
            with self.subTest(error=error):
                observed = []

                def make_runtime(*args, **kwargs):
                    path = real_mkdtemp(*args, dir=self.root, **kwargs)
                    observed.append(Path(path))
                    return path

                options = ["--repo", str(self.repo), "--virtual-gamepad",
                           "--simulator", sys.executable, "--executor", sys.executable,
                           "--log-dir", str(self.root / "logs")]
                with mock.patch.object(launcher, "generate_scene"), \
                        mock.patch.object(launcher.tempfile, "mkdtemp", side_effect=make_runtime), \
                        mock.patch.object(launcher, "run_session", side_effect=OSError("test failure") if error else None,
                                          return_value=0), \
                        contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(launcher.main(options), 1 if error else 0)
                self.assertEqual(len(observed), 1)
                self.assertFalse(observed[0].exists())

    def test_duel_script_imports_when_called_outside_repository(self):
        result = subprocess.run([sys.executable, "-B", str(launcher.REPO_ROOT / "tools/duel_sim/launcher.py"),
                                 "--help"], cwd=self.root, text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()

"""No hardware, graphics, SDK libraries, or third-party Python packages required."""

import contextlib
import ast
import hashlib
import io
import os
import shutil
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

from tools.duel_sim import launcher


class DuelLauncherTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.assets = launcher.REPO_ROOT / "assets"

    def tearDown(self):
        self.temp.cleanup()

    def session(self, *options):
        args = launcher.parse_args(["--dry-run", *options])
        return launcher.build_session(args, self.root / "runtime", self.root / "logs")

    def test_scene_has_independent_names_and_matching_references(self):
        path = launcher.generate_scene(self.assets, self.root / "duel.xml")
        scene = ET.parse(path).getroot()
        self.assertEqual(len(scene.findall(".//freejoint")), 2)
        self.assertFalse(scene.findall(".//include"))
        names = {element.attrib["name"] for element in scene.iter() if "name" in element.attrib}
        classes = {element.attrib["class"] for element in scene.iter("default") if "class" in element.attrib}
        for element in scene.iter():
            for key, value in element.attrib.items():
                if key in launcher.REFERENCE_ATTRIBUTES:
                    self.assertIn(value, names | classes, (element.tag, key, value))
            if "file" in element.attrib:
                asset = Path(element.attrib["file"])
                self.assertTrue(asset.is_absolute())
                self.assertTrue(asset.is_file(), asset)
        source = launcher.expand_xml(self.assets / "resource/robot/t800/xml/serial_t800.xml")
        original_motors = [element.attrib["name"] for element in source.findall("actuator/motor")]
        original_sensors = [element.attrib["name"] for element in source.findall("sensor/*")]
        motors = [element.attrib["name"] for element in scene.findall("actuator/motor")]
        sensors = [element.attrib["name"] for element in scene.findall("sensor/*")]
        for index, prefix in enumerate(launcher.ROBOT_PREFIXES):
            self.assertEqual(motors[index * 25:(index + 1) * 25], [prefix + name for name in original_motors])
            count = len(original_sensors)
            self.assertEqual(sensors[index * count:(index + 1) * count], [prefix + name for name in original_sensors])
        self.assertEqual(len(motors), 50)

    def test_robots_face_each_other_with_feet_near_ground(self):
        scene = ET.parse(launcher.generate_scene(self.assets, self.root / "duel.xml")).getroot()
        first = scene.find("worldbody/body[@name='robot_a/LINK_BASE']")
        second = scene.find("worldbody/body[@name='robot_b/LINK_BASE']")
        self.assertEqual(first.attrib["quat"], "1 0 0 0")
        self.assertEqual(second.attrib["quat"], "0 0 0 1")
        self.assertEqual(float(first.attrib["pos"].split()[0]), -1)
        self.assertEqual(float(second.attrib["pos"].split()[0]), 1)
        self.assertAlmostEqual(float(first.attrib["pos"].split()[2]), 1.02155, places=5)

    def test_overlays_isolate_ports_inputs_ros_and_logs_without_source_writes(self):
        config = self.assets / "config/t800"
        protected = [config / path for path in ("mode.yaml", "lcm/default.yaml", "ros2_bridge/default.yaml", "google_logger/default.yaml", "pd_stand/default.yaml")]
        before = [hashlib.sha256(path.read_bytes()).digest() for path in protected]
        plans = self.session("--gamepad-a", "/dev/input/by-id/player_a-joystick",
                             "--gamepad-b", "/dev/input/by-id/player_b-joystick")
        sim, first, second = plans
        self.assertEqual(sim["command"][-2:], ["t800", "--duel"])
        for index, plan in enumerate((first, second)):
            env = plan["env"]
            overlay = Path(env["ENGINEAI_ROBOTICS_ASSETS"])
            text = (overlay / "config/t800/mode.yaml").read_text()
            self.assertTrue(text.startswith("active_mode: sim\n"))
            self.assertIn(f"239.255.76.67:{17667 + index}", (overlay / "config/t800/lcm/default.yaml").read_text())
            self.assertEqual(env["ENGINEAI_INPUT_SOURCE"], "gamepad")
            self.assertEqual(env["ROS_DOMAIN_ID"], str(70 + index))
            name = "robot_a" if index == 0 else "robot_b"
            self.assertIn(f"/{name}/hardware/joint_state", (overlay / "config/t800/ros2_bridge/default.yaml").read_text())
            self.assertIn(str(self.root / "logs" / name), (overlay / "config/t800/google_logger/default.yaml").read_text())
            combined = overlay / "config/t800/rl_walking_leolab_ankle_combined_v2_140000"
            self.assertFalse(combined.is_symlink())
            self.assertTrue((combined / "default.yaml").is_file())
            self.assertTrue((combined / "policies/boxing_policy.mnn").is_file())
            policy = overlay / "config/t800/rl_straight_punch_L_improved/policies/policy.mnn"
            self.assertTrue(policy.is_symlink())
            self.assertEqual(policy.resolve(), (config / policy.relative_to(overlay / "config/t800")).resolve())
            self.assertTrue((overlay / "resource").is_symlink())
            self.assertFalse((overlay / "config/t800/lcm/default.yaml").is_symlink())
        self.assertNotEqual(first["env"]["ENGINEAI_GAMEPAD_DEVICE"], second["env"]["ENGINEAI_GAMEPAD_DEVICE"])
        self.assertEqual(before, [hashlib.sha256(path.read_bytes()).digest() for path in protected])

    def test_all_yaml_scopes_are_visible_without_following_symlinks(self):
        original = self.assets / "config/t800"
        expected = {path.relative_to(original) for path in original.rglob("*.yaml")}
        for plan in self.session()[1:]:
            config = Path(plan["env"]["ENGINEAI_ROBOTICS_ASSETS"]) / "config/t800"
            # Match the SDK's directory scan: ignore linked directories/files.
            visible = {Path(directory, name).relative_to(config)
                       for directory, _, names in os.walk(config, followlinks=False)
                       for name in names if name.endswith(".yaml")
                       and not Path(directory, name).is_symlink()}
            self.assertEqual(visible, expected)
            self.assertIn(Path("model/default.yaml"), visible)

    def test_virtual_gamepad_controls_only_a_on_its_selected_port(self):
        sim, first, second, virtual = self.session(
            "--virtual-gamepad", "--port-a", "27667", "--port-b", "27668",
            "--gamepad-a", "/dev/input/js0", "--gamepad-b", "/dev/input/js0")
        self.assertEqual(first["env"]["ENGINEAI_INPUT_SOURCE"], "virtual")
        self.assertNotIn("ENGINEAI_GAMEPAD_DEVICE", first["env"])
        self.assertEqual(second["env"]["ENGINEAI_INPUT_SOURCE"], "gamepad")
        self.assertEqual(second["env"]["ENGINEAI_GAMEPAD_DEVICE"], "/dev/input/js0")
        self.assertEqual(virtual["name"], "virtual_gamepad")
        self.assertEqual(virtual["command"][-3:],
                         ["--lcm-url", "udpm://239.255.76.67:27667?ttl=0", "--connect"])
        for plan, port in ((first, 27667), (second, 27668)):
            config = Path(plan["env"]["ENGINEAI_ROBOTICS_CONFIG"])
            self.assertIn(f"239.255.76.67:{port}", (config / "t800/lcm/default.yaml").read_text())
        self.assertEqual(len({plan["log"] for plan in (sim, first, second, virtual)}), 4)

    def test_virtual_launch_does_not_require_either_physical_gamepad(self):
        output = io.StringIO()
        with mock.patch.object(launcher, "run_session", return_value=0) as run, \
                contextlib.redirect_stdout(output), contextlib.redirect_stderr(io.StringIO()):
            code = launcher.main([
                "--virtual-gamepad", "--gamepad-a", str(self.root / "missing_a"),
                "--gamepad-b", str(self.root / "missing_b"),
                "--simulator", sys.executable, "--executor", sys.executable,
                "--log-dir", str(self.root / "logs")])
        self.assertEqual(code, 0)
        run.assert_called_once()
        self.assertIn("Robot B has no readable gamepad", output.getvalue())

    def test_virtual_gamepad_requires_controllers(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            launcher.parse_args(["--virtual-gamepad", "--no-controllers"])

    def test_private_pd_target_matches_zero_pose_and_preserves_gains_and_timing(self):
        original_path = self.assets / "config/t800/pd_stand/default.yaml"
        original = original_path.read_text()
        original_rows = [ast.literal_eval(line.split("- ", 1)[1]) for line in original.split("stiffness:", 1)[0].splitlines()
                         if line.lstrip().startswith("- ")]
        plans = self.session()
        for plan in plans[1:]:
            config = Path(plan["env"]["ENGINEAI_ROBOTICS_ASSETS"]) / "config/t800"
            pd_file = config / "pd_stand/default.yaml"
            self.assertFalse(pd_file.is_symlink())
            pd_text = pd_file.read_text()
            rows = [ast.literal_eval(line.split("- ", 1)[1]) for line in pd_text.split("stiffness:", 1)[0].splitlines()
                    if line.lstrip().startswith("- ")]
            self.assertEqual([len(row) for row in rows], [len(row) for row in original_rows])
            self.assertEqual(sum(map(len, rows)), 25)
            self.assertTrue(all(value == 0.0 for row in rows for value in row))
            self.assertEqual(pd_text.split("stiffness:", 1)[1], original.split("stiffness:", 1)[1])
            for sibling in original_path.parent.iterdir():
                if sibling.name != "default.yaml":
                    target = config / "pd_stand" / sibling.name
                    self.assertFalse(target.is_symlink())
                    self.assertEqual(target.read_bytes(), sibling.read_bytes())
            self.assertIn("scope: pd_stand/default", (config / "mode.yaml").read_text())
            self.assertIn("scope: pd_stand/pose_x", (config / "mode.yaml").read_text())
        self.assertEqual(original_path.read_text(), original)

    def test_pd_target_parser_rejects_unsupported_or_non_numeric_rows(self):
        for text in ("stiffness: 3\n", "desired_joint_position:\n", "desired_joint_position:\n  - 4\n",
                     "desired_joint_position:\n  - [True]\n", "desired_joint_position:\n  - [1e999]\n",
                     "desired_joint_position:\n  - []\n", "desired_joint_position:\n  - [invalid]\n",
                     "desired_joint_position:\n  - [0]\ndesired_joint_position:\n  - [0]\n"):
            with self.subTest(text=text), self.assertRaisesRegex(ValueError, "PD"):
                launcher.zero_pd_target(text)

    def test_headless_scene_only_has_no_controller_processes(self):
        plans = self.session("--headless", "--steps", "20", "--no-controllers")
        self.assertEqual(len(plans), 1)
        self.assertEqual(plans[0]["command"][-3:], ["--headless", "--steps", "20"])

    def test_overlay_forces_sim_when_original_is_robot(self):
        fixture_assets = self.root / "fixture/assets"
        config = fixture_assets / "config/t800"
        shutil.copytree(self.assets / "config/t800", config, symlinks=True,
                        copy_function=launcher.copy_config_file)
        (fixture_assets / "resource").symlink_to(self.assets / "resource", target_is_directory=True)
        original = (self.assets / "config/t800/mode.yaml").read_text().replace("active_mode: sim", "active_mode: robot", 1)
        (config / "mode.yaml").write_text(original)
        overlay = launcher.create_overlay(fixture_assets, self.root / "overlay", "robot_a", 17667, self.root / "logs")
        self.assertTrue((overlay / "config/t800/mode.yaml").read_text().startswith("active_mode: sim\n"))
        self.assertEqual((config / "mode.yaml").read_text(), original)

    def test_linked_private_directory_cannot_write_into_source_configuration(self):
        fixture_assets = self.root / "fixture/assets"
        config = fixture_assets / "config/t800"
        config.mkdir(parents=True)
        source = self.assets / "config/t800/lcm"
        before = (source / "default.yaml").read_bytes()
        (config / "lcm").symlink_to(source, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "real directory: lcm"):
            launcher.create_overlay(fixture_assets, self.root / "overlay", "robot_a", 27667,
                                    self.root / "logs")
        self.assertEqual((source / "default.yaml").read_bytes(), before)

    def test_linked_private_file_cannot_write_into_source_configuration(self):
        fixture_assets = self.root / "fixture/assets"
        config = fixture_assets / "config/t800"
        (config / "lcm").mkdir(parents=True)
        source = self.assets / "config/t800/lcm/default.yaml"
        before = source.read_bytes()
        (config / "lcm/default.yaml").symlink_to(source)
        with self.assertRaisesRegex(ValueError, "real file: lcm/default.yaml"):
            launcher.create_overlay(fixture_assets, self.root / "overlay", "robot_a", 27667,
                                    self.root / "logs")
        self.assertEqual(source.read_bytes(), before)

    def test_dry_run_cleans_generated_configuration(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(launcher.main(["--dry-run"]), 0)
        import json
        summary = json.loads(output.getvalue())
        self.assertEqual(len(summary["processes"]), 3)
        self.assertFalse(Path(summary["runtime"]).exists())
        for plan in summary["processes"]:
            self.assertNotIn("ENGINEAI_ROBOTICS_COPYRIGHT", plan["env"])

    def test_reject_duplicate_devices_and_invalid_configuration(self):
        for options in (("--gamepad-a", "/dev/input/js0", "--gamepad-b", "/dev/input/js0"),
                        ("--port-a", "17668"), ("--port-a", "100000"),
                        ("--steps", "20"), ("--headless", "--steps", "0"),
                        ("--separation", "nan"), ("--ros-domain-a", "232")):
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    launcher.parse_args(options)
        device = self.root / "device"
        device.touch()
        alias = self.root / "alias"
        alias.symlink_to(device)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            launcher.parse_args(["--gamepad-a", str(device), "--gamepad-b", str(alias)])

    def test_recursive_include_cycle_reports_error(self):
        path = self.root / "cycle.xml"
        path.write_text('<mujoco><include file="cycle.xml"/></mujoco>')
        with self.assertRaisesRegex(ValueError, "Cyclic MJCF include"):
            launcher.expand_xml(path)

    def test_child_failure_stops_other_owned_process(self):
        pid_file = self.root / "sleeper.pid"
        pid_writer = f"import os,time; open({str(pid_file)!r},'w').write(str(os.getpid())); time.sleep(30)"
        plans = [
            {"name": "sleeper", "command": [sys.executable, "-c", pid_writer], "env": os.environ.copy(), "log": self.root / "sleeper.log"},
            {"name": "failure", "command": [sys.executable, "-c", "import time; time.sleep(.2); raise SystemExit(7)"], "env": os.environ.copy(), "log": self.root / "failure.log"},
        ]
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(launcher.run_session(plans, launcher.REPO_ROOT), 7)
        pid = int(pid_file.read_text())
        with self.assertRaises(ProcessLookupError):
            os.kill(pid, 0)

    def test_spawn_failure_also_stops_previously_started_child(self):
        plans = [
            {"name": "sleeper", "command": [sys.executable, "-c", "import time; time.sleep(30)"], "env": os.environ.copy(), "log": self.root / "sleeper.log"},
            {"name": "missing", "command": [str(self.root / "not-an-executable")], "env": os.environ.copy(), "log": self.root / "missing.log"},
        ]
        with self.assertRaises(FileNotFoundError):
            launcher.run_session(plans, launcher.REPO_ROOT)


if __name__ == "__main__":
    unittest.main()

"""Single-launcher tests without SDK libraries, graphics, or a physical gamepad."""

import contextlib
import hashlib
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

from tools.duel_sim import launcher, single_launcher


class SingleLauncherTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.repo = self.root / "repo"
        self.config = self.repo / "assets/config/t800"
        self.config.mkdir(parents=True)
        (self.config / "mode.yaml").write_text("active_mode: sim\n")
        (self.config / "pd_stand").mkdir()
        (self.config / "pd_stand/default.yaml").write_text(
            "desired_joint_position:\n  - [0.4, -0.8]\nstiffness: 1080\n")
        (self.config / "lcm").mkdir()
        (self.config / "lcm/default.yaml").write_text(
            "multicast: false\nip_port: 239.255.76.67:7667\nttl: 1\n")

    def options(self, *options):
        return ["--repo", str(self.repo), *options]

    def session(self, *options):
        args = single_launcher.parse_args(self.options(*options))
        return single_launcher.build_session(args, self.root / "logs")

    def snapshot(self):
        return {p.relative_to(self.config): hashlib.sha256(p.read_bytes()).digest()
                for p in self.config.rglob("*") if p.is_file()}

    def test_single_session_uses_original_assets_and_preserves_configuration(self):
        before = self.snapshot()
        sim, robot = self.session("--gamepad", "/dev/input/by-id/player-joystick")
        assets = str(self.repo / "assets")
        self.assertEqual(sim["command"][1:], ["t800"])
        self.assertEqual(robot["command"][1:], ["t800"])
        for plan in (sim, robot):
            self.assertEqual(plan["env"]["ENGINEAI_ROBOTICS_ASSETS"], assets)
            self.assertEqual(plan["env"]["ENGINEAI_ROBOTICS_CONFIG"], assets + "/config")
            self.assertEqual(plan["env"]["ENGINEAI_ROBOTICS_PLATFORM"], "mujoco")
            self.assertEqual(plan["env"]["ENGINEAI_ROBOTICS_RUNTIME_ENV"], "sim")
            self.assertNotIn("--duel", plan["command"])
        self.assertEqual(robot["env"]["ENGINEAI_GAMEPAD_DEVICE"],
                         "/dev/input/by-id/player-joystick")
        self.assertEqual(robot["env"]["ENGINEAI_INPUT_SOURCE"], "gamepad")
        self.assertNotEqual(sim["log"], robot["log"])
        self.assertEqual(before, self.snapshot())

    def test_robot_mode_is_rejected_without_editing_the_source(self):
        mode = self.config / "mode.yaml"
        mode.write_text("active_mode: robot\n")
        before = self.snapshot()
        with self.assertRaises(ValueError):
            self.session()
        with contextlib.redirect_stderr(io.StringIO()), contextlib.redirect_stdout(io.StringIO()):
            self.assertNotEqual(single_launcher.main(self.options("--dry-run")), 0)
        self.assertEqual(before, self.snapshot())

    def test_virtual_session_is_device_independent_and_preserves_configuration(self):
        before = self.snapshot()
        with mock.patch.dict(os.environ, {"ENGINEAI_GAMEPAD_DEVICE": "/dev/input/inherited"}):
            sim, robot, virtual = self.session("--virtual-gamepad")
        self.assertEqual(robot["env"]["ENGINEAI_INPUT_SOURCE"], "virtual")
        self.assertNotIn("ENGINEAI_GAMEPAD_DEVICE", robot["env"])
        self.assertEqual(virtual["command"][-3:],
                         ["--lcm-url", "udpm://239.255.76.67:7667?ttl=0", "--connect"])
        self.assertEqual(virtual["command"][0], sys.executable)
        self.assertEqual(len({plan["log"] for plan in (sim, robot, virtual)}), 3)
        self.assertEqual(before, self.snapshot())

    def test_virtual_launch_does_not_require_a_physical_gamepad(self):
        with mock.patch.object(single_launcher, "existing_sdk_processes", return_value=[]), \
                mock.patch.object(single_launcher, "run_session", return_value=0) as run, \
                contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            code = single_launcher.main(self.options(
                "--virtual-gamepad", "--gamepad", str(self.root / "missing"),
                "--simulator", sys.executable, "--executor", sys.executable,
                "--log-dir", str(self.root / "logs")))
        self.assertEqual(code, 0)
        run.assert_called_once()

    def test_virtual_gamepad_requires_a_controller(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            single_launcher.parse_args(self.options("--virtual-gamepad", "--no-controllers"))

    def test_scene_only_headless_needs_no_gamepad_or_executor(self):
        (sim,) = self.session("--no-controllers", "--headless", "--steps", "20")
        self.assertEqual(sim["command"][1:], ["t800", "--headless", "--steps", "20"])

    def test_dry_run_prints_both_plans_without_sdk_or_devices(self):
        before = self.snapshot()
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            code = single_launcher.main(self.options("--dry-run"))
        self.assertEqual(code, 0)
        summary = json.loads(output.getvalue())
        self.assertEqual(len(summary["processes"]), 2)
        self.assertEqual(before, self.snapshot())

    def test_rejects_invalid_headless_step_requests(self):
        for options in (("--steps", "20"), ("--headless", "--steps", "0"),
                        ("--headless", "--steps", "-1")):
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    single_launcher.parse_args(self.options(*options))

    def test_existing_sdk_instance_is_reported_without_starting_children(self):
        output = io.StringIO()
        with mock.patch.object(single_launcher, "existing_sdk_processes",
                               return_value=["4242 (src_executor)"]), \
                mock.patch.object(single_launcher, "run_session") as run, \
                contextlib.redirect_stderr(output), contextlib.redirect_stdout(io.StringIO()):
            code = single_launcher.main(self.options())
        self.assertNotEqual(code, 0)
        self.assertIn("4242", output.getvalue())
        run.assert_not_called()

    def executable(self, name, body):
        path = self.root / name
        path.write_text(f"#!{sys.executable}\n" + body)
        path.chmod(0o755)
        return path

    def launcher_command(self, *options):
        # Isolate only the read-only busy-instance check: the integration still
        # runs real children and the production signal/cleanup implementation.
        code = ("import sys; from unittest.mock import patch; "
                "from tools.duel_sim import single_launcher; "
                "p=patch.object(single_launcher, 'existing_sdk_processes', return_value=[]); "
                "p.start(); raise SystemExit(single_launcher.main(sys.argv[1:]))")
        device = self.root / "joystick"
        device.touch()
        return [sys.executable, "-c", code, *self.options(
            "--gamepad", str(device), "--log-dir", str(self.root / "process_logs"), *options)]

    def sleeper(self, name):
        pid_file = self.root / (name + ".pid")
        body = ("import os,time\n"
                f"open({str(pid_file)!r}, 'w').write(str(os.getpid()))\n"
                "time.sleep(30)\n")
        self.addCleanup(self.cleanup_pid_file, pid_file)
        return self.executable(name, body), pid_file

    @staticmethod
    def cleanup_pid_file(path):
        if path.exists():
            try:
                os.kill(int(path.read_text()), signal.SIGTERM)
            except ProcessLookupError:
                pass

    def unrelated_process(self):
        process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])

        def cleanup():
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=5)

        self.addCleanup(cleanup)
        return process

    def wait_for_pids(self, process, *paths):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if all(p.exists() and p.read_text().strip() for p in paths):
                return
            if process.poll() is not None:
                self.fail(f"launcher exited before children started: {process.returncode}")
            time.sleep(0.02)
        self.fail("timed out waiting for test children")

    def assert_stopped(self, path):
        self.assertTrue(path.exists())
        with self.assertRaises(ProcessLookupError):
            os.kill(int(path.read_text()), 0)

    def test_controller_failure_stops_its_simulator_and_preserves_other_processes(self):
        simulator, sim_pid = self.sleeper("fake_simulator")
        controller = self.executable("failing_executor",
            "import pathlib,time\n"
            f"p=pathlib.Path({str(sim_pid)!r})\n"
            "deadline=time.monotonic()+5\n"
            "while not p.exists() and time.monotonic()<deadline: time.sleep(.02)\n"
            "raise SystemExit(7)\n")
        unrelated = self.unrelated_process()
        result = subprocess.run(self.launcher_command("--simulator", str(simulator),
            "--executor", str(controller)), cwd=launcher.REPO_ROOT,
            capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 7, result.stderr)
        self.assert_stopped(sim_pid)
        self.assertIsNone(unrelated.poll())

    def test_sigint_stops_both_owned_children_and_preserves_other_processes(self):
        simulator, sim_pid = self.sleeper("fake_simulator")
        executor, executor_pid = self.sleeper("fake_executor")
        unrelated = self.unrelated_process()
        process = subprocess.Popen(self.launcher_command("--simulator", str(simulator),
            "--executor", str(executor)), cwd=launcher.REPO_ROOT,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)

        def cleanup():
            if process.poll() is None:
                process.terminate()
            process.communicate(timeout=10)

        self.addCleanup(cleanup)
        self.wait_for_pids(process, sim_pid, executor_pid)
        process.send_signal(signal.SIGINT)
        _, stderr = process.communicate(timeout=10)
        self.assertEqual(process.returncode, 130, stderr)
        self.assert_stopped(sim_pid)
        self.assert_stopped(executor_pid)
        self.assertIsNone(unrelated.poll())


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""Launch the original T800 simulation and one controller together."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import sys
import tempfile

from .launcher import (
    DISPLAY_ENV_KEYS, REPO_ROOT, child_environment, copy_assets_config,
    run_session, virtual_gamepad_plan,
)
from .numbered_actions import configure_numbered_actions


def build_session(args, log_dir: Path, runtime: Path | None = None) -> list[dict]:
    repo = args.repo.resolve()
    assets = repo / "assets"
    mode_path = assets / "config/t800/mode.yaml"
    mode = mode_path.read_text()
    # Keep the original config intact and reject a hardware mode before launch.
    headers = re.findall(r"(?m)^active_mode:[^\r\n]*", mode)
    if len(headers) != 1 or not re.fullmatch(
        r"active_mode:[ \t]*(?:sim|'sim'|\"sim\")[ \t]*(?:#[^\r\n]*)?", headers[0]
    ):
        raise ValueError(f"Single simulation requires active_mode: sim in {mode_path}")

    catalog = None
    if args.virtual_gamepad and (assets / "config/t800/numbered_actions/catalog.json").exists():
        assets = copy_assets_config(assets, (runtime or log_dir / "runtime") / "assets")
        catalog = configure_numbered_actions(assets / "config/t800")

    sim_command = [str(args.simulator.resolve()), "t800"]
    if args.headless:
        sim_command.append("--headless")
    if args.steps is not None:
        sim_command.extend(("--steps", str(args.steps)))
    plans = []
    for name, command in [("simulation", sim_command), ("robot", [str(args.executor.resolve()), "t800"])]:
        if name == "robot" and args.no_controllers:
            continue
        process_logs = log_dir / name
        process_logs.mkdir(parents=True, exist_ok=True)
        env = child_environment(repo, assets, process_logs)
        if name == "robot":
            if args.virtual_gamepad:
                env["ENGINEAI_INPUT_SOURCE"] = "virtual"
                env.pop("ENGINEAI_GAMEPAD_DEVICE", None)
            else:
                env["ENGINEAI_GAMEPAD_DEVICE"] = args.gamepad
        plans.append({"name": name, "command": command, "env": env,
                      "log": process_logs / "console.log"})
    if args.virtual_gamepad:
        plans.append(virtual_gamepad_plan(repo, log_dir, "udpm://239.255.76.67:7667?ttl=0", catalog))
    return plans


def existing_sdk_processes() -> list[str]:
    """Find SDK binaries without matching shell commands or signalling anyone."""
    running = []
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            name = (process / "exe").readlink().name.removesuffix(" (deleted)")
        except OSError:
            continue
        if name in {"src_executor", "engineai_robotics_simulation_mujoco"}:
            running.append(f"{process.name} ({name})")
    return running


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=REPO_ROOT, help=argparse.SUPPRESS)
    parser.add_argument("--gamepad", default="/dev/input/js0", help="Linux joystick device for this robot")
    parser.add_argument("--virtual-gamepad", action="store_true",
                        help="open a virtual gamepad instead of requiring a physical device")
    parser.add_argument("--simulator", type=Path,
                        default=REPO_ROOT / "simulation/mujoco/build/engineai_robotics_simulation_mujoco")
    parser.add_argument("--executor", type=Path, default=REPO_ROOT / "build/x86_64/_install/bin/src_executor")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps", type=int, help="stop after this many headless simulation steps")
    parser.add_argument("--no-controllers", action="store_true", help="start only the single-robot scene")
    parser.add_argument("--dry-run", action="store_true", help="print commands without launching processes")
    parser.add_argument("--log-dir", type=Path, help="process logs (default: a new /tmp/engineai-single-logs-* directory)")
    args = parser.parse_args(argv)
    if args.steps is not None and (args.steps <= 0 or not args.headless):
        parser.error("--steps requires --headless and a positive step count")
    if args.virtual_gamepad and args.no_controllers:
        parser.error("--virtual-gamepad requires a controller")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    preview_logs = None
    runtime = None
    try:
        if not args.dry_run:
            running = existing_sdk_processes()
            if running:
                raise ValueError("SDK processes are already running; exit their launcher first: " + ", ".join(running))
        if args.log_dir:
            log_dir = args.log_dir.resolve()
        elif args.dry_run:
            preview_logs = tempfile.TemporaryDirectory(prefix="engineai-single-preview-")
            log_dir = Path(preview_logs.name)
        else:
            log_dir = Path(tempfile.mkdtemp(prefix="engineai-single-logs-"))
        if args.virtual_gamepad and (args.repo / "assets/config/t800/numbered_actions/catalog.json").exists():
            runtime = tempfile.TemporaryDirectory(prefix="engineai-single-")
        plans = build_session(args, log_dir, Path(runtime.name) if runtime else None)
        summary = [{"name": plan["name"], "command": plan["command"],
                    "env": {key: value for key, value in plan["env"].items() if key in DISPLAY_ENV_KEYS},
                    "log": str(plan["log"])} for plan in plans]
        print(json.dumps({"processes": summary}, ensure_ascii=False, indent=2), flush=True)
        if args.dry_run:
            return 0
        for plan in plans:
            binary = Path(plan["command"][0])
            if not binary.is_file() or not os.access(binary, os.X_OK):
                raise ValueError(f"Build the executable first: {binary}")
        if not args.no_controllers and not args.virtual_gamepad and (not Path(args.gamepad).exists() or not os.access(args.gamepad, os.R_OK)):
            raise ValueError(f"Gamepad is missing or unreadable: {args.gamepad}")
        print(f"[INFO] Process logs: {log_dir}", flush=True)
        print("[INFO] Using the original T800 single-robot scene, PD targets and state/action keys.", flush=True)
        if args.virtual_gamepad:
            print("[INFO] Virtual gamepad controls this robot. Press F5/Enable in its window to send input.", flush=True)
        return run_session(plans, args.repo.resolve())
    except (OSError, ValueError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 1
    finally:
        if runtime is not None:
            runtime.cleanup()
        if preview_logs is not None:
            preview_logs.cleanup()


if __name__ == "__main__":
    sys.exit(main())

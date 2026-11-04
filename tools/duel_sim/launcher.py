#!/usr/bin/env python3
"""Launch two isolated T800 controllers in one MuJoCo world.

Scene generation and configuration overlays use only the Python standard library.
All source assets remain read-only; writable configuration belongs to this session.
"""

from __future__ import annotations

import argparse
import ast
import copy
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[2]
ROBOT_PREFIXES = ("robot_a/", "robot_b/")
DISPLAY_ENV_KEYS = {
    "ENGINEAI_ROBOTICS_DIR", "ENGINEAI_ROBOTICS_ASSETS", "ENGINEAI_ROBOTICS_CONFIG",
    "ENGINEAI_ROBOTICS_PLATFORM", "ENGINEAI_ROBOTICS_RUNTIME_ENV", "ENGINEAI_INPUT_SOURCE",
    "ENGINEAI_GAMEPAD_DEVICE", "ENGINEAI_DUEL_SCENE", "ENGINEAI_DUEL_PORT_A", "ENGINEAI_DUEL_PORT_B",
    "ROS_DOMAIN_ID", "ROS_LOG_DIR",
}
REFERENCE_ATTRIBUTES = {
    "class", "childclass", "mesh", "material", "texture", "hfield", "body", "body1", "body2",
    "joint", "joint1", "joint2", "site", "site1", "site2", "geom", "geom1", "geom2", "tendon",
    "tendon1", "tendon2", "actuator", "target", "objname", "refname", "cranksite", "slidersite",
}


def expand_xml(path: Path, stack: tuple[Path, ...] = ()) -> ET.Element:
    """Expand MJCF include files and resolve each asset relative to its own file."""
    path = path.resolve(strict=True)
    if path in stack:
        raise ValueError(f"Cyclic MJCF include: {path}")
    root = ET.parse(path).getroot()

    def expand(element: ET.Element) -> None:
        for child in list(element):
            if child.tag == "include":
                included = expand_xml(path.parent / child.attrib["file"], (*stack, path))
                index = list(element).index(child)
                element.remove(child)
                for offset, included_child in enumerate(included):
                    element.insert(index + offset, included_child)
            else:
                if "file" in child.attrib:
                    child.set("file", str((path.parent / child.attrib["file"]).resolve(strict=True)))
                expand(child)

    expand(root)
    return root


def prefix_robot(source: ET.Element, prefix: str) -> ET.Element:
    robot = copy.deepcopy(source)
    names = {element.attrib["name"] for element in robot.iter() if "name" in element.attrib}
    classes = {element.attrib["class"] for element in robot.iter("default") if "class" in element.attrib}
    for element in robot.iter():
        for attribute, value in list(element.attrib.items()):
            if attribute == "name" or (attribute in REFERENCE_ATTRIBUTES and value in names | classes):
                element.set(attribute, prefix + value)
    return robot


def standing_base_height(robot: ET.Element, clearance: float = 0.005) -> float:
    """Place the shipped T800's zero-pose front-foot boxes just above the floor."""
    root_body = robot.find("worldbody/body[@name='LINK_BASE']")
    if root_body is None:
        raise ValueError("The duel scene requires the T800 LINK_BASE body")
    parents = {child: parent for parent in root_body.iter() for child in parent}
    sole_heights = []
    for side in ("L", "R"):
        foot = root_body.find(f".//body[@name='LINK_FOOT_{side}']")
        side_name = "left" if side == "L" else "right"
        default = robot.find(f".//default[@class='collision_{side_name}_fore_foot']/geom")
        if foot is None or default is None:
            raise ValueError(f"Missing {side_name} T800 foot collision definition")
        height = float(default.attrib["pos"].split()[2]) - float(default.attrib["size"].split()[2])
        body = foot
        while body is not root_body:
            if "quat" in body.attrib or "euler" in body.attrib:
                raise ValueError("Automatic T800 foot placement expects unrotated body frames")
            height += float(body.attrib.get("pos", "0 0 0").split()[2])
            body = parents[body]
        sole_heights.append(height)
    return clearance - min(sole_heights)


def generate_scene(assets: Path, destination: Path, separation: float = 2.0) -> Path:
    source = expand_xml(assets / "resource/robot/t800/xml/serial_t800.xml")
    ground = expand_xml(assets / "resource/environment/ground.xml")
    height = standing_base_height(source)
    scene = ET.Element("mujoco", model="t800_duel")
    # Global compiler/solver settings occur once; each robot owns its definitions.
    for tag in ("compiler", "option"):
        element = source.find(tag)
        if element is not None:
            scene.append(copy.deepcopy(element))
    for element in ground:
        if element.tag == "statistic":
            element.set("center", "0 0 0.8")
            element.set("extent", "3")
        scene.append(element)
    for index, prefix in enumerate(ROBOT_PREFIXES):
        robot = prefix_robot(source, prefix)
        base = robot.find(f"worldbody/body[@name='{prefix}LINK_BASE']")
        assert base is not None
        x = (-0.5 if index == 0 else 0.5) * separation
        base.set("pos", f"{x:.8g} 0 {height:.8g}")
        base.set("quat", "1 0 0 0" if index == 0 else "0 0 0 1")
        for element in robot:
            if element.tag not in ("compiler", "option", "keyframe"):
                scene.append(element)
    ET.indent(scene)
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(scene).write(destination, encoding="utf-8", xml_declaration=True)
    return destination


def replace_config(source: Path, destination: Path, transform) -> None:
    """Copy one config into a private directory, symlinking only its siblings."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    for sibling in source.parent.iterdir():
        target = destination.parent / sibling.name
        if sibling.name != source.name and not target.exists():
            target.symlink_to(sibling, target_is_directory=sibling.is_dir())
    destination.write_text(transform(source.read_text()), encoding="utf-8")


def zero_pd_target(text: str) -> str:
    """Keep PD gains/timing intact and hold the duel world's zero joint pose."""
    lines = text.splitlines(keepends=True)
    headers = [index for index, line in enumerate(lines)
               if re.fullmatch(r"desired_joint_position:[ \t]*(?:#[^\r\n]*)?\r?\n?", line)]
    if len(headers) != 1:
        raise ValueError("PD config must contain one desired_joint_position array")
    count = 0
    for index in range(headers[0] + 1, len(lines)):
        line = lines[index]
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if not line[0].isspace():
            break
        row = re.fullmatch(r"([ \t]+-[ \t]*)(\[[^\r\n]*\])([ \t]*(?:#[^\r\n]*)?)(\r?\n)?", line)
        if row is None:
            raise ValueError("PD desired_joint_position must use one numeric list per limb")
        try:
            values = ast.literal_eval(row[2])
        except (ValueError, SyntaxError) as error:
            raise ValueError("PD desired_joint_position contains an invalid numeric list") from error
        if (not isinstance(values, list) or not values
                or any(type(value) not in (int, float) or not math.isfinite(value) for value in values)):
            raise ValueError("PD desired_joint_position requires nonempty lists of finite numbers")
        lines[index] = row[1] + str([0.0] * len(values)) + row[3] + (row[4] or "")
        count += 1
    if count == 0:
        raise ValueError("PD desired_joint_position has no limb arrays")
    return "".join(lines)


def create_overlay(assets: Path, destination: Path, name: str, port: int, log_dir: Path) -> Path:
    config = assets / "config/t800"
    target = destination / "config/t800"
    target.mkdir(parents=True)
    for child in assets.iterdir():
        if child.name != "config":
            (destination / child.name).symlink_to(child, target_is_directory=child.is_dir())
    private_dirs = {"lcm", "ros2_bridge", "google_logger", "pd_stand"}
    for child in config.iterdir():
        if child.name != "mode.yaml" and child.name not in private_dirs:
            (target / child.name).symlink_to(child, target_is_directory=child.is_dir())
    mode = config.joinpath("mode.yaml").read_text()
    mode, count = re.subn(r"(?m)^active_mode:\s*\S+\s*$", "active_mode: sim", mode, count=1)
    if count != 1:
        raise ValueError("Cannot select simulation mode in t800/mode.yaml")
    for tag in sorted(private_dirs):
        mode = re.sub(
            rf"(- tag: {tag}\s*\n\s*scope:)\s*\S+", rf"\1 {tag}/default", mode
        )
    (target / "mode.yaml").write_text(mode, encoding="utf-8")
    (target / "lcm").mkdir()
    (target / "lcm/default.yaml").write_text(
        f"multicast: false\nip_port: 239.255.76.67:{port}\nttl: 0\n", encoding="utf-8"
    )
    replace_config(
        config / "ros2_bridge/default.yaml", target / "ros2_bridge/default.yaml",
        lambda text: re.sub(r"(?m)^(\s+\w+:\s+)(/\S+)(\s*)$", rf"\1/{name}\2\3", text),
    )
    replace_config(
        config / "google_logger/default.yaml", target / "google_logger/default.yaml",
        lambda text: re.sub(r"(?m)^glog_dir:.*$", f"glog_dir: {json.dumps(str(log_dir))}", text),
    )
    replace_config(config / "pd_stand/default.yaml", target / "pd_stand/default.yaml", zero_pd_target)
    return destination


def child_environment(repo: Path, assets: Path, log_dir: Path) -> dict[str, str]:
    env = os.environ.copy()
    env.update(
        ENGINEAI_ROBOTICS_DIR=str(repo), ENGINEAI_ROBOTICS_ASSETS=str(assets),
        ENGINEAI_ROBOTICS_CONFIG=str(assets / "config"), ENGINEAI_ROBOTICS_PLATFORM="mujoco",
        ENGINEAI_ROBOTICS_RUNTIME_ENV="sim", ENGINEAI_INPUT_SOURCE="gamepad",
        ROS_LOG_DIR=str(log_dir / "ros"),
    )
    third_party = Path(env.get("ENGINEAI_ROBOTICS_THIRD_PARTY", "/opt/engineai_robotics_third_party"))
    hardware = Path(env.get("ENGINEAI_ROBOTICS_HARDWARE", "/opt/engineai_robotics_hardware"))
    env["ENGINEAI_ROBOTICS_THIRD_PARTY"] = str(third_party)
    env["ENGINEAI_ROBOTICS_HARDWARE"] = str(hardware)
    lib_paths = [repo / "build/x86_64/_install/lib", repo / "core/lib", third_party / "lib",
                 third_party / "lib/runtime", hardware / "lib"]
    env["LD_LIBRARY_PATH"] = ":".join([*map(str, lib_paths), *filter(None, [env.get("LD_LIBRARY_PATH", "")])])
    return env


def build_session(args, runtime: Path, log_dir: Path) -> list[dict]:
    repo = args.repo.resolve()
    assets = repo / "assets"
    scene = generate_scene(assets, runtime / "t800_duel.xml", args.separation)
    plans = []
    overlays = []
    for index, (name, port, device) in enumerate(zip(
        ("robot_a", "robot_b"), (args.port_a, args.port_b), (args.gamepad_a, args.gamepad_b)
    )):
        process_logs = log_dir / name
        process_logs.mkdir(parents=True, exist_ok=True)
        overlay = create_overlay(assets, runtime / name / "assets", name, port, process_logs)
        overlays.append(overlay)
        if not args.no_controllers:
            env = child_environment(repo, overlay, process_logs)
            env.update(ENGINEAI_GAMEPAD_DEVICE=device, ROS_DOMAIN_ID=str(args.ros_domain_a + index))
            plans.append({"name": name, "command": [str(args.executor.resolve()), "t800"],
                          "env": env, "log": process_logs / "console.log"})
    sim_logs = log_dir / "simulation"
    sim_logs.mkdir(parents=True, exist_ok=True)
    env = child_environment(repo, overlays[0], sim_logs)
    env.update(ENGINEAI_DUEL_SCENE=str(scene), ENGINEAI_DUEL_PORT_A=str(args.port_a),
               ENGINEAI_DUEL_PORT_B=str(args.port_b))
    command = [str(args.simulator.resolve()), "t800", "--duel"]
    if args.headless:
        command.append("--headless")
    if args.steps is not None:
        command.extend(("--steps", str(args.steps)))
    plans.insert(0, {"name": "simulation", "command": command, "env": env,
                     "log": sim_logs / "console.log"})
    return plans


def stop_children(processes: list[subprocess.Popen], timeout: float = 5.0) -> None:
    # Each child has its own session. Never signal a process outside these groups.
    for process in processes:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + timeout
    for process in processes:
        try:
            process.wait(timeout=max(0.0, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()


def run_session(plans: list[dict], repo: Path) -> int:
    processes = []
    logs = []
    previous_handlers = {}

    def interrupted(signum, _frame):
        raise KeyboardInterrupt

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, interrupted)
        for plan in plans:
            stream = plan["log"].open("w")
            logs.append(stream)
            process = subprocess.Popen(plan["command"], cwd=repo, env=plan["env"], stdout=stream,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(process)
        while True:
            for plan, process in zip(plans, processes):
                code = process.poll()
                if code is not None:
                    if code:
                        print(f"[ERROR] {plan['name']} exited with {code}: {plan['log']}", file=sys.stderr)
                        print(plan["log"].read_text(errors="replace")[-6000:], file=sys.stderr)
                    return code if code >= 0 else 128 - code
            time.sleep(0.05)
    except KeyboardInterrupt:
        return 130
    finally:
        for signum in previous_handlers:
            signal.signal(signum, signal.SIG_IGN)
        stop_children(processes)
        for stream in logs:
            stream.close()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=REPO_ROOT, help=argparse.SUPPRESS)
    parser.add_argument("--gamepad-a", default="/dev/input/js0")
    parser.add_argument("--gamepad-b", default="/dev/input/js1")
    parser.add_argument("--port-a", type=int, default=17667)
    parser.add_argument("--port-b", type=int, default=17668)
    parser.add_argument("--ros-domain-a", type=int, default=70,
                        help="A uses this ROS domain, B uses the next one (default: 70/71)")
    parser.add_argument("--separation", type=float, default=2.0, help="initial base separation in metres")
    parser.add_argument("--simulator", type=Path,
                        default=REPO_ROOT / "simulation/mujoco/build/engineai_robotics_simulation_mujoco")
    parser.add_argument("--executor", type=Path, default=REPO_ROOT / "build/x86_64/_install/bin/src_executor")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps", type=int, help="stop after this many headless simulation steps")
    parser.add_argument("--no-controllers", action="store_true", help="run the shared scene without controllers")
    parser.add_argument("--dry-run", action="store_true", help="print private configuration and commands without launching")
    parser.add_argument("--log-dir", type=Path, help="persistent process logs (default: a new /tmp/engineai-duel-logs-* directory)")
    parser.add_argument("--keep-runtime", action="store_true", help="retain generated scene/config after exit for inspection")
    args = parser.parse_args(argv)
    if not all(1024 <= port <= 65535 for port in (args.port_a, args.port_b)) or args.port_a == args.port_b:
        parser.error("ports must be distinct numbers between 1024 and 65535")
    if not 0 <= args.ros_domain_a <= 231:
        parser.error("--ros-domain-a must be between 0 and 231")
    if not math.isfinite(args.separation) or args.separation < 1.0:
        parser.error("--separation must be finite and at least 1 metre")
    if args.steps is not None and (args.steps <= 0 or not args.headless):
        parser.error("--steps requires --headless and a positive step count")
    if not args.no_controllers and Path(args.gamepad_a).resolve() == Path(args.gamepad_b).resolve():
        parser.error("two different gamepad devices are required")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    runtime = Path(tempfile.mkdtemp(prefix="engineai-duel-"))
    try:
        if args.log_dir:
            log_dir = args.log_dir.resolve()
        elif args.dry_run:
            log_dir = runtime / "logs"
        else:
            log_dir = Path(tempfile.mkdtemp(prefix="engineai-duel-logs-"))
        plans = build_session(args, runtime, log_dir)
        summary = []
        for plan in plans:
            selected_env = {key: value for key, value in plan["env"].items() if key in DISPLAY_ENV_KEYS}
            summary.append({"name": plan["name"], "command": plan["command"], "env": selected_env,
                            "log": str(plan["log"])})
        print(json.dumps({"runtime": str(runtime), "processes": summary}, ensure_ascii=False, indent=2), flush=True)
        if args.dry_run:
            return 0
        for plan in plans:
            binary = Path(plan["command"][0])
            if not binary.is_file() or not os.access(binary, os.X_OK):
                raise ValueError(f"Build the executable first: {binary}")
        if not args.no_controllers:
            for device in (args.gamepad_a, args.gamepad_b):
                if not Path(device).exists() or not os.access(device, os.R_OK):
                    raise ValueError(f"Gamepad is missing or unreadable: {device}")
        print(f"[INFO] Process logs: {log_dir}", flush=True)
        print("[INFO] Each gamepad controls one robot using the existing T800 state/action keys.", flush=True)
        return run_session(plans, args.repo.resolve())
    except (OSError, ValueError, ET.ParseError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 1
    finally:
        if args.keep_runtime:
            print(f"[INFO] Generated scene/config retained: {runtime}", flush=True)
        else:
            shutil.rmtree(runtime)


if __name__ == "__main__":
    sys.exit(main())

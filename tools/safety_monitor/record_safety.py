#!/usr/bin/env python3
"""Read-only T800 safety telemetry recorder.

This process subscribes to existing ROS2 topics and the optional LCM task_state
channel. It records IMU posture context for walking-motion limit events. It
never publishes commands and never changes robot state.
"""

from __future__ import annotations

import argparse
import datetime as datetime_lib
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as element_tree
from typing import Any, Optional

import yaml


# lcm-gen encodes the fingerprint returned by TaskState::getHash(), which is
# the one-bit left rotation of the seed in TaskState::_computeHash().
LCM_TASK_STATE_HASH = 0x104561AA026A2B0E
LCM_SIM_STATE_HASH = 0xADB53C89E035F05C
LCM_DYNAMIC_VECTOR_DOUBLE_HASH = 0xD22DAF2A4F45DA70
DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=0"
IMU_GRAVITY_MPS2 = 9.80665
WALK_MOTIONS = frozenset(
    {
        "walk",
        "walk_leo",
        "walk_v2_dr_194500",
        "walk_v2_dr_191500",
        "walk_plus_origin_180000",
        "walk_plus_dr_189000",
        "leo_taunt",
    }
)
LEO_MOTION_PARAM_TAGS = {
    "walk_leo": "rl_walking_leolab_example",
    "walk_v2_dr_194500": "rl_walking_leolab_impact_v2_dr_194500",
    "walk_v2_dr_191500": "rl_walking_leolab_impact_v2_dr_191500",
    "walk_plus_origin_180000": "rl_walking_leolab_impact_v2_plus_origin_180000",
    "walk_plus_dr_189000": "rl_walking_leolab_impact_v2_plus_dr_189000",
    "leo_taunt": "rl_walking_leolab_taunt",
}
POSTURE_EVENT_TYPES = frozenset(
    {
        "model_position_limit_violation",
        "model_position_limit_near",
        "model_velocity_limit_violation",
        "model_effort_limit_violation",
    }
)


def utc_now() -> str:
    return datetime_lib.datetime.now(datetime_lib.timezone.utc).isoformat(timespec="milliseconds")


def finite_float(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def finite_float_list(values: Any) -> list[Optional[float]]:
    return [finite_float(value) for value in values]


def vector3_record(value: Any) -> list[Optional[float]]:
    if value is None:
        return [None, None, None]
    return finite_float_list(
        [
            getattr(value, "x", None),
            getattr(value, "y", None),
            getattr(value, "z", None),
        ]
    )


def vector_norm(values: list[Optional[float]]) -> Optional[float]:
    if len(values) < 3 or any(value is None for value in values[:3]):
        return None
    return math.sqrt(sum(float(value) ** 2 for value in values[:3]))


def leo_command_diagnostics_record(message: Any, monotonic_ns: int, ros_time_ns: int) -> dict[str, Any]:
    """Record the runner's command speed chain without mixing clock domains."""
    return {
        "receive_monotonic_ns": monotonic_ns,
        "receive_ros_time_ns": ros_time_ns,
        "ros_stamp_ns": message_stamp_ns(message),
        "source_monotonic_ns": int(getattr(message, "source_monotonic_ns", 0)),
        "source_param_tag": str(getattr(message, "source_param_tag", "")),
        "active": bool(getattr(message, "active", False)),
        "raw_stick": finite_float_list(getattr(message, "raw_stick", [])),
        "target_tactical": finite_float_list(getattr(message, "target_tactical", [])),
        "shaped_tactical": finite_float_list(getattr(message, "shaped_tactical", [])),
        "policy_command": finite_float_list(getattr(message, "policy_command", [])),
        "source_age_ms": finite_float(getattr(message, "source_age_ms", None)),
    }


def is_new_leo_command_diagnostics(record: dict[str, Any], previous_source_ns: Optional[int]) -> bool:
    source_ns = record.get("source_monotonic_ns")
    return isinstance(source_ns, int) and source_ns > (previous_source_ns or 0)


def leo_command_diagnostics_freshness(
    record: Optional[dict[str, Any]], sample_monotonic_ns: int,
    max_age_ms: Optional[float] = 100.0, motion: Optional[str] = None,
) -> dict[str, Any]:
    if record is None:
        return {"available": False, "fresh": False, "age_ms": None}
    receive_ns = record.get("receive_monotonic_ns")
    receive_age_ms = (
        (sample_monotonic_ns - receive_ns) / 1_000_000.0
        if isinstance(receive_ns, int) and receive_ns >= 0
        else None
    )
    source_age_ms = finite_float(record.get("source_age_ms"))
    # The publisher measures source age on its own host. Add elapsed receipt
    # time on this host; never subtract two machines' monotonic timestamps.
    # Transport delay is not measured, so this is an age estimate, not a bound.
    valid_ages = (receive_age_ms is not None and receive_age_ms >= 0.0
                  and source_age_ms is not None and source_age_ms >= 0.0)
    age_ms = receive_age_ms + source_age_ms if valid_ages else None
    expected_tag = LEO_MOTION_PARAM_TAGS.get(motion)
    motion_matches = expected_tag is not None and record.get("source_param_tag") == expected_tag
    active = record.get("active") is True
    return {
        "available": True,
        "fresh": (active and motion_matches and age_ms is not None
                  and (max_age_ms is None or age_ms <= max_age_ms)),
        "active": active,
        "motion_matches": motion_matches,
        "age_ms": age_ms,
        "receive_age_ms": receive_age_ms,
        "source_age_ms": source_age_ms,
    }


def policy_diagnostics_freshness(
    record: Optional[dict[str, Any]], sample_monotonic_ns: int,
    max_age_ms: Optional[float] = 100.0, motion: Optional[str] = None,
) -> dict[str, Any]:
    """Validate that a cached policy snapshot belongs to this event sample."""
    if record is None:
        return {
            "available": False, "fresh": False, "age_ms": None,
            "active": False, "policy_output_valid": False,
            "motion_matches": False, "reason": "policy_diagnostics_unavailable",
        }

    receive_ns = record.get("receive_monotonic_ns")
    age_ms = (
        (sample_monotonic_ns - int(receive_ns)) / 1_000_000.0
        if isinstance(receive_ns, int) else None
    )
    expected_tag = LEO_MOTION_PARAM_TAGS.get(motion)
    source_tag = record.get("source_param_tag")
    motion_matches = expected_tag is not None and source_tag == expected_tag
    active = record.get("active") is True
    output_valid = record.get("policy_output_valid") is True
    raw_action = record.get("raw_action")
    action_available = (
        isinstance(raw_action, list) and len(raw_action) > 0
        and all(value is not None for value in raw_action)
    )
    age_valid = age_ms is not None and age_ms >= 0.0 and (
        max_age_ms is None or age_ms <= max_age_ms
    )
    fresh = active and output_valid and motion_matches and action_available and age_valid
    if fresh:
        reason = "fresh"
    elif not age_valid:
        reason = "stale_or_missing_receive_time"
    elif not active:
        reason = "policy_inactive"
    elif not output_valid:
        reason = "policy_output_invalid"
    elif not motion_matches:
        reason = "policy_param_tag_mismatch"
    else:
        reason = "policy_action_unavailable"
    return {
        "available": True,
        "fresh": fresh,
        "age_ms": age_ms,
        "active": active,
        "policy_output_valid": output_valid,
        "motion_matches": motion_matches,
        "source_param_tag": source_tag,
        "expected_param_tag": expected_tag,
        "reason": reason,
    }


def quaternion_tilt_rad(quaternion: list[Optional[float]]) -> Optional[float]:
    if len(quaternion) < 4 or any(value is None for value in quaternion[:4]):
        return None
    w, x, y, z = (float(value) for value in quaternion[:4])
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-12:
        return None
    x /= norm
    y /= norm
    vertical_component = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
    return math.acos(vertical_component)


def imu_record(message: Any, monotonic_ns: int, ros_time_ns: int) -> dict[str, Any]:
    quaternion_message = getattr(message, "quaternion", None)
    quaternion = finite_float_list(
        [
            getattr(quaternion_message, "w", None),
            getattr(quaternion_message, "x", None),
            getattr(quaternion_message, "y", None),
            getattr(quaternion_message, "z", None),
        ]
    )
    rpy = vector3_record(getattr(message, "rpy", None))
    linear_acceleration = vector3_record(getattr(message, "linear_acceleration", None))
    angular_velocity = vector3_record(getattr(message, "angular_velocity", None))
    tilt_rad = quaternion_tilt_rad(quaternion)
    if tilt_rad is None and rpy[0] is not None and rpy[1] is not None:
        tilt_rad = math.hypot(float(rpy[0]), float(rpy[1]))
    acceleration_norm = vector_norm(linear_acceleration)
    angular_speed = vector_norm(angular_velocity)
    acceleration_deviation = (
        abs(acceleration_norm - IMU_GRAVITY_MPS2) if acceleration_norm is not None else None
    )
    return {
        "receive_monotonic_ns": monotonic_ns,
        "receive_ros_time_ns": ros_time_ns,
        "ros_stamp_ns": message_stamp_ns(message),
        "quaternion_wxyz": quaternion,
        "rpy": rpy,
        "linear_acceleration_mps2": linear_acceleration,
        "angular_velocity_rad_s": angular_velocity,
        "tilt_rad": tilt_rad,
        "tilt_deg": math.degrees(tilt_rad) if tilt_rad is not None else None,
        "acceleration_norm_mps2": acceleration_norm,
        "acceleration_deviation_mps2": acceleration_deviation,
        "angular_speed_rad_s": angular_speed,
    }


def int_list(values: Any) -> list[Optional[int]]:
    result: list[Optional[int]] = []
    for value in values:
        try:
            result.append(int(value))
        except (TypeError, ValueError):
            result.append(None)
    return result


def message_stamp_ns(message: Any) -> Optional[int]:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is None:
        return None
    try:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None


def resolve_path(value: str, repo_root: Path) -> Path:
    path = Path(value).expanduser()
    return (repo_root / path).resolve() if not path.is_absolute() else path.resolve()


def sha256_file(path: Path) -> Optional[str]:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as file_handle:
        for block in iter(lambda: file_handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_output(repo_root: Path, *arguments: str) -> str:
    try:
        result = subprocess.run(
            ["git", *arguments],
            cwd=repo_root,
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    return result.stdout.strip()


def load_joint_names(model_yaml: Path) -> list[str]:
    document = yaml.safe_load(model_yaml.read_text(encoding="utf-8")) or {}
    names = [
        str(joint_name)
        for limb in document.get("limbs", [])
        for joint_name in limb.get("joints", [])
    ]
    if not names:
        raise ValueError(f"No joints found in model YAML: {model_yaml}")
    if len(names) != len(set(names)):
        raise ValueError(f"Duplicate joint names found in model YAML: {model_yaml}")
    return names


def load_urdf_limits(urdf_path: Path) -> dict[str, dict[str, Optional[float]]]:
    root = element_tree.parse(urdf_path).getroot()
    limits: dict[str, dict[str, Optional[float]]] = {}
    for joint in root.findall(".//joint"):
        if joint.attrib.get("type") == "fixed":
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        limits[joint.attrib["name"]] = {
            key: finite_float(limit.attrib.get(key))
            for key in ("lower", "upper", "effort", "velocity")
        }
    return limits


def lcm_url_from_config(config_path: Path) -> str:
    if not config_path.is_file():
        return DEFAULT_LCM_URL
    try:
        document = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return DEFAULT_LCM_URL
    if document.get("url"):
        return str(document["url"])
    ip_port = document.get("ip_port")
    if not ip_port:
        return DEFAULT_LCM_URL
    if str(ip_port).startswith("udpm://"):
        return str(ip_port)
    multicast = bool(document.get("multicast", False))
    ttl = int(document.get("ttl", 1)) if multicast else 0
    return f"udpm://{ip_port}?ttl={ttl}"


def decode_lcm_task_state(payload: bytes) -> Optional[str]:
    if len(payload) < 12:
        return None
    try:
        message_hash = struct.unpack(">q", payload[:8])[0]
        string_length = struct.unpack(">i", payload[8:12])[0]
    except struct.error:
        return None
    if message_hash != LCM_TASK_STATE_HASH or string_length < 1:
        return None
    end = 12 + string_length
    if end > len(payload):
        return None
    try:
        return payload[12 : end - 1].decode("utf-8")
    except UnicodeDecodeError:
        return payload[12 : end - 1].decode("utf-8", errors="replace")


def decode_lcm_sim_state(payload: bytes) -> Optional[dict[str, Any]]:
    """Decode the generated SimState message without requiring lcm-gen Python bindings."""
    if len(payload) < 8 or struct.unpack(">Q", payload[:8])[0] != LCM_SIM_STATE_HASH:
        return None
    offset = 8

    def read_double() -> float:
        nonlocal offset
        value = struct.unpack_from(">d", payload, offset)[0]
        offset += 8
        return value

    def read_int() -> int:
        nonlocal offset
        value = struct.unpack_from(">i", payload, offset)[0]
        offset += 4
        return value

    def read_doubles(count: int) -> list[float]:
        nonlocal offset
        if count < 0 or count > 10000:
            raise ValueError("invalid SimState vector length")
        values = list(struct.unpack_from(f">{count}d", payload, offset)) if count else []
        offset += count * 8
        return values

    try:
        timestamp = read_double()
        joint_count = read_int()
        joint_position = read_doubles(joint_count)
        joint_velocity = read_doubles(joint_count)
        joint_torque = read_doubles(joint_count)
        base_position = read_doubles(3)
        base_linear_velocity = read_doubles(3)
        base_quaternion = read_doubles(4)
        base_angular_velocity = read_doubles(3)
        imu_link_position = read_doubles(3)
        imu_link_linear_velocity = read_doubles(3)
        imu_link_quaternion = read_doubles(4)
        imu_link_angular_velocity = read_doubles(3)
        imu_sensor_quaternion = read_doubles(4)
        imu_sensor_linear_acceleration = read_doubles(3)
        imu_sensor_angular_velocity = read_doubles(3)
        contact_count = read_int()
        contact_force = read_doubles(contact_count)
    except (IndexError, struct.error, ValueError):
        return None
    return {
        "receive_monotonic_ns": time.monotonic_ns(),
        "receive_ros_time_ns": None,
        "sim_time_s": timestamp,
        "joint_position": finite_float_list(joint_position),
        "joint_velocity": finite_float_list(joint_velocity),
        "joint_torque": finite_float_list(joint_torque),
        "base_position": finite_float_list(base_position),
        "base_linear_velocity": finite_float_list(base_linear_velocity),
        "base_quaternion_wxyz": finite_float_list(base_quaternion),
        "base_angular_velocity": finite_float_list(base_angular_velocity),
        "imu_link_position": finite_float_list(imu_link_position),
        "imu_link_linear_velocity": finite_float_list(imu_link_linear_velocity),
        "imu_link_quaternion_wxyz": finite_float_list(imu_link_quaternion),
        "imu_link_angular_velocity": finite_float_list(imu_link_angular_velocity),
        "imu_sensor_quaternion_wxyz": finite_float_list(imu_sensor_quaternion),
        "imu_sensor_linear_acceleration": finite_float_list(imu_sensor_linear_acceleration),
        "imu_sensor_angular_velocity": finite_float_list(imu_sensor_angular_velocity),
        "contact_force": finite_float_list(contact_force),
        "source": "lcm_sim_state",
    }


def decode_lcm_dynamic_vector(payload: bytes) -> Optional[dict[str, Any]]:
    if len(payload) < 20 or struct.unpack(">Q", payload[:8])[0] != LCM_DYNAMIC_VECTOR_DOUBLE_HASH:
        return None
    try:
        timestamp = struct.unpack_from(">d", payload, 8)[0]
        count = struct.unpack_from(">i", payload, 16)[0]
        if count < 0 or count > 10000 or len(payload) < 20 + count * 8:
            return None
        values = struct.unpack_from(f">{count}d", payload, 20) if count else ()
    except struct.error:
        return None
    if count < 2 or (count - 2) % 3 != 0:
        return None
    num_joints = (count - 2) // 3
    return {
        "receive_monotonic_ns": time.monotonic_ns(),
        "receive_ros_time_ns": None,
        "sim_time_s": timestamp,
        "nominal_ctrl": finite_float_list(values[:num_joints]),
        "effective_ctrl": finite_float_list(values[num_joints:2 * num_joints]),
        "injected_noise": finite_float_list(values[2 * num_joints:3 * num_joints]),
        "command_age_ms": finite_float(values[3 * num_joints]),
        "command_valid": bool(values[3 * num_joints + 1]),
        "source": "lcm_sim_control_diagnostics",
    }


class SafetyRecorder:
    def __init__(
        self,
        node: Any,
        args: argparse.Namespace,
        message_types: tuple[Any, Any, Any, Any, Any, Any, Any, Any, Any, Any],
        joint_names: list[str],
        limits: dict[str, dict[str, Optional[float]]],
        qos_profile: Any,
    ) -> None:
        self.node = node
        self.args = args
        self.joint_names = joint_names
        self.limits = limits
        self.lock = threading.RLock()
        self.closed = False
        self.sample_count = 0
        self.event_record_count = 0
        self.next_event_id = 1
        self.active: dict[str, dict[str, Any]] = {}
        self.latest: dict[str, dict[str, Any]] = {}
        self.motion: Optional[str] = None
        self.motion_source: Optional[str] = None
        self.motion_time_ns: Optional[int] = None
        self.available_transitions: list[str] = []
        self.last_sample: Optional[dict[str, Any]] = None
        self.last_joint_receive_ns: Optional[int] = None

        self.output_dir = Path(args.output)
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.samples_path = self.output_dir / "samples.jsonl"
        self.events_path = self.output_dir / "events.jsonl"
        self.metadata_path = self.output_dir / "metadata.json"
        self.samples_file = self.samples_path.open("w", encoding="utf-8", buffering=1)
        self.events_file = self.events_path.open("w", encoding="utf-8", buffering=1)

        config_paths = [Path(path) for path in args.config]
        if args.lcm_config:
            config_paths.append(Path(args.lcm_config))
        self.metadata: dict[str, Any] = {
            "schema_version": "t800_safety_log_v3",
            "read_only_recorder": True,
            "started_at_utc": utc_now(),
            "hostname": socket.gethostname(),
            "pid": os.getpid(),
            "repo_root": str(args.repo_root),
            "git_branch": git_output(args.repo_root, "branch", "--show-current"),
            "git_commit": git_output(args.repo_root, "rev-parse", "HEAD"),
            "git_status": git_output(args.repo_root, "status", "--short", "--branch"),
            "model_yaml": str(args.model_yaml),
            "urdf": str(args.urdf),
            "configs": [str(path) for path in config_paths],
            "config_sha256": {str(path): sha256_file(path) for path in config_paths},
            "joint_names": joint_names,
            "joint_count": len(joint_names),
            "urdf_limit_count": len(limits),
            "urdf_limits": limits,
            "topics": {
                "joint_state": args.joint_state_topic,
                "motor_state": args.motor_state_topic,
                "joint_command_feedback": args.joint_command_topic,
                "motor_command": args.motor_command_topic,
                "motor_debug": args.motor_debug_topic,
                "power_info": args.power_info_topic,
                "motion_state": args.motion_topic,
                "imu_info": args.imu_topic,
                "gamepad_keys": args.gamepad_keys_topic,
                "rc02_diagnostics": args.rc02_diagnostics_topic,
                "policy_diagnostics": args.policy_diagnostics_topic,
                "leo_command_diagnostics": args.leo_command_diagnostics_topic,
            },
            "input_sources": {
                "mujoco": "hardware/gamepad_keys (virtual controller or Xbox)",
                "real_robot": "hardware/rc02_diagnostics (paired RC02, when enabled)",
            },
            "lcm": {
                "enabled": not args.no_lcm,
                "url": args.lcm_url,
                "channel": args.lcm_channel,
                "sim_state_channel": args.lcm_sim_state_channel,
                "sim_state_enabled": not args.no_lcm and not args.no_sim_state,
                "control_diagnostics_channel": args.lcm_control_diagnostics_channel,
                "control_diagnostics_enabled": not args.no_lcm and not args.no_control_diagnostics,
            },
            "timestamp_semantics": {
                "receive_monotonic_ns": "recorder_host_steady_clock",
                "receive_ros_time_ns": "recorder_node_clock",
                "ros_stamp_ns": "message_header_stamp",
                "sim_time_s": "mujoco_simulation_clock",
                "source_monotonic_ns": "publisher_host_steady_clock",
            },
            "thresholds": {
                "position_margin_rad": args.position_margin,
                "tracking_error_rad": args.tracking_error_threshold,
                "expected_joint_rate_hz": args.expected_joint_rate,
                "gap_factor": args.gap_factor,
                "imu_max_age_ms": args.imu_max_age_ms,
                "leo_command_max_age_ms": args.leo_command_max_age_ms,
                "posture_tilt_threshold_deg": args.posture_tilt_threshold_deg,
                "posture_fallen_threshold_deg": args.posture_fallen_threshold_deg,
                "posture_angular_speed_threshold_rad_s": args.posture_angular_speed_threshold,
                "posture_acceleration_deviation_threshold_mps2": args.posture_acceleration_deviation,
            },
            "files": {
                "samples": str(self.samples_path),
                "events": str(self.events_path),
            },
        }
        self.write_metadata()

        (
            joint_state_type,
            joint_command_type,
            motor_debug_type,
            power_info_type,
            motion_state_type,
            imu_info_type,
            leo_command_diagnostics_type,
            gamepad_keys_type,
            rc02_diagnostics_type,
            policy_diagnostics_type,
        ) = message_types
        self.subscriptions = [
            node.create_subscription(joint_state_type, args.joint_state_topic, self.on_joint_state, qos_profile),
            node.create_subscription(joint_state_type, args.motor_state_topic, self.on_motor_state, qos_profile),
            node.create_subscription(joint_command_type, args.joint_command_topic, self.on_joint_command, qos_profile),
            node.create_subscription(joint_command_type, args.motor_command_topic, self.on_motor_command, qos_profile),
            node.create_subscription(motor_debug_type, args.motor_debug_topic, self.on_motor_debug, qos_profile),
            node.create_subscription(power_info_type, args.power_info_topic, self.on_power_info, qos_profile),
            node.create_subscription(imu_info_type, args.imu_topic, self.on_imu_info, qos_profile),
            node.create_subscription(gamepad_keys_type, args.gamepad_keys_topic, self.on_gamepad_keys, qos_profile),
        ]
        if rc02_diagnostics_type is not None:
            self.subscriptions.append(
                node.create_subscription(rc02_diagnostics_type, args.rc02_diagnostics_topic,
                                         self.on_rc02_diagnostics, qos_profile)
            )
        if policy_diagnostics_type is not None:
            self.subscriptions.append(
                node.create_subscription(policy_diagnostics_type, args.policy_diagnostics_topic,
                                         self.on_policy_diagnostics, qos_profile)
            )
        if motion_state_type is not None:
            self.subscriptions.append(
                node.create_subscription(motion_state_type, args.motion_topic, self.on_motion_state, qos_profile)
            )
        if leo_command_diagnostics_type is not None:
            self.subscriptions.append(
                node.create_subscription(
                    leo_command_diagnostics_type,
                    args.leo_command_diagnostics_topic,
                    self.on_leo_command_diagnostics,
                    qos_profile,
                )
            )

        self.duration_deadline_ns = (
            time.monotonic_ns() + int(args.duration * 1_000_000_000)
            if args.duration is not None
            else None
        )
        self.lcm = None
        self.lcm_thread: Optional[threading.Thread] = None
        self.lcm_stop = threading.Event()
        self.lcm_error: Optional[str] = None
        if not args.no_lcm:
            self.start_lcm()

    def receipt_times(self) -> tuple[int, int]:
        return time.monotonic_ns(), int(self.node.get_clock().now().nanoseconds)

    def write_metadata(self) -> None:
        temporary_path = self.metadata_path.with_suffix(".json.tmp")
        temporary_path.write_text(
            json.dumps(self.metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary_path.replace(self.metadata_path)

    def write_line(self, file_handle: Any, record: dict[str, Any], force_sync: bool = False) -> None:
        file_handle.write(json.dumps(record, ensure_ascii=False, separators=(",", ":"), allow_nan=False))
        file_handle.write("\n")
        file_handle.flush()
        periodic_sync = (
            self.args.fsync_every > 0
            and self.sample_count > 0
            and self.sample_count % self.args.fsync_every == 0
        )
        if force_sync or periodic_sync:
            os.fsync(file_handle.fileno())

    def state_record(self, message: Any, monotonic_ns: int, ros_time_ns: int) -> dict[str, Any]:
        return {
            "receive_monotonic_ns": monotonic_ns,
            "receive_ros_time_ns": ros_time_ns,
            "ros_stamp_ns": message_stamp_ns(message),
            "position": finite_float_list(message.position),
            "velocity": finite_float_list(message.velocity),
            "torque": finite_float_list(message.torque),
        }

    def command_record(self, message: Any, monotonic_ns: int, ros_time_ns: int) -> dict[str, Any]:
        return {
            **self.state_record(message, monotonic_ns, ros_time_ns),
            "feed_forward_torque": finite_float_list(message.feed_forward_torque),
            "stiffness": finite_float_list(message.stiffness),
            "damping": finite_float_list(message.damping),
            "parallel_parser_type": int(getattr(message, "parallel_parser_type", 0)),
        }

    def on_joint_state(self, message: Any) -> None:
        monotonic_ns, ros_time_ns = self.receipt_times()
        with self.lock:
            if self.closed:
                return
            previous_ns = self.last_joint_receive_ns
            self.last_joint_receive_ns = monotonic_ns
            state = self.state_record(message, monotonic_ns, ros_time_ns)
            self.latest["joint_state"] = state
            self.sample_count += 1
            period_ms = None if previous_ns is None else (monotonic_ns - previous_ns) / 1_000_000.0
            motion_age_ms = (
                None
                if self.motion_time_ns is None
                else (monotonic_ns - self.motion_time_ns) / 1_000_000.0
            )
            leo_diagnostics = self.copy_latest("leo_command_diagnostics")
            if leo_diagnostics is not None:
                leo_diagnostics["freshness"] = leo_command_diagnostics_freshness(
                    leo_diagnostics, monotonic_ns, self.args.leo_command_max_age_ms, self.motion
                )
            sample = {
                "schema_version": 3,
                "sample_index": self.sample_count,
                "receive_monotonic_ns": monotonic_ns,
                "receive_ros_time_ns": ros_time_ns,
                "ros_stamp_ns": state["ros_stamp_ns"],
                "joint_state_period_ms": period_ms,
                "motion": self.motion,
                "motion_source": self.motion_source,
                "motion_age_ms": motion_age_ms,
                "available_transition_motions": self.available_transitions,
                "joint_state": state,
                "motor_state": self.copy_latest("motor_state"),
                "joint_command_feedback": self.copy_latest("joint_command_feedback"),
                "motor_command": self.copy_latest("motor_command"),
                "motor_debug": self.copy_latest("motor_debug"),
                "power_info": self.copy_latest("power_info"),
                "imu_info": self.copy_latest("imu_info"),
                "gamepad_keys": self.copy_latest("gamepad_keys"),
                "rc02_diagnostics": self.copy_latest("rc02_diagnostics"),
                "policy_diagnostics": self.copy_latest("policy_diagnostics"),
                "sim_state": self.copy_latest("sim_state"),
                "sim_control_diagnostics": self.copy_latest("sim_control_diagnostics"),
                "leo_command_diagnostics": leo_diagnostics,
            }
            self.last_sample = sample
            self.write_line(self.samples_file, sample)
            if period_ms is not None:
                expected_ms = 1000.0 / self.args.expected_joint_rate
                if period_ms > expected_ms * self.args.gap_factor:
                    self.emit_instant(
                        "joint_state_gap",
                        {"gap_ms": period_ms, "expected_period_ms": expected_ms},
                        sample,
                    )
            self.evaluate(sample)

    def on_motor_state(self, message: Any) -> None:
        monotonic_ns, ros_time_ns = self.receipt_times()
        with self.lock:
            if not self.closed:
                self.latest["motor_state"] = self.state_record(message, monotonic_ns, ros_time_ns)

    def on_joint_command(self, message: Any) -> None:
        monotonic_ns, ros_time_ns = self.receipt_times()
        with self.lock:
            if not self.closed:
                self.latest["joint_command_feedback"] = self.command_record(message, monotonic_ns, ros_time_ns)

    def on_motor_command(self, message: Any) -> None:
        monotonic_ns, ros_time_ns = self.receipt_times()
        with self.lock:
            if not self.closed:
                self.latest["motor_command"] = self.command_record(message, monotonic_ns, ros_time_ns)

    def on_motor_debug(self, message: Any) -> None:
        monotonic_ns, ros_time_ns = self.receipt_times()
        record = {
            "receive_monotonic_ns": monotonic_ns,
            "receive_ros_time_ns": ros_time_ns,
            "mos_temperature": finite_float_list(message.mos_temperature),
            "motor_temperature": finite_float_list(message.motor_temperature),
            "voltage": finite_float_list(message.voltage),
            "current": finite_float_list(message.current),
            "error_code": int_list(message.error_code),
            "offline": int_list(message.offline),
            "enable": int_list(message.enable),
        }
        with self.lock:
            if not self.closed:
                self.latest["motor_debug"] = record

    def on_power_info(self, message: Any) -> None:
        monotonic_ns, ros_time_ns = self.receipt_times()
        record = {
            "receive_monotonic_ns": monotonic_ns,
            "receive_ros_time_ns": ros_time_ns,
            "enable": bool(message.enable),
            "percentage": finite_float(message.percentage),
            "voltage": finite_float(message.voltage),
            "current": finite_float(message.current),
            "current_limit": finite_float(message.current_limit),
            "error_code": int(message.error_code),
        }
        with self.lock:
            if not self.closed:
                self.latest["power_info"] = record

    def on_imu_info(self, message: Any) -> None:
        monotonic_ns, ros_time_ns = self.receipt_times()
        with self.lock:
            if not self.closed:
                self.latest["imu_info"] = imu_record(message, monotonic_ns, ros_time_ns)

    def on_gamepad_keys(self, message: Any) -> None:
        monotonic_ns, ros_time_ns = self.receipt_times()
        record = {
            "receive_monotonic_ns": monotonic_ns,
            "receive_ros_time_ns": ros_time_ns,
            "ros_stamp_ns": message_stamp_ns(message),
            "hardware_connected": bool(getattr(message, "hardware_connected", False)),
            "digital_states": int_list(getattr(message, "digital_states", [])),
            "analog_states": finite_float_list(getattr(message, "analog_states", [])),
            "source": "ros2_gamepad_keys",
        }
        with self.lock:
            if not self.closed:
                self.latest["gamepad_keys"] = record

    def on_rc02_diagnostics(self, message: Any) -> None:
        monotonic_ns, ros_time_ns = self.receipt_times()
        record = {
            "receive_monotonic_ns": monotonic_ns,
            "receive_ros_time_ns": ros_time_ns,
            "ros_stamp_ns": message_stamp_ns(message),
            "raw_frame_size": int(getattr(message, "raw_frame_size", 0)),
            "raw_frame": int_list(getattr(message, "raw_frame", [])),
            "source_monotonic_ns": int(getattr(message, "source_monotonic_ns", 0)),
            "connected": bool(getattr(message, "connected", False)),
            "frame_valid": bool(getattr(message, "frame_valid", False)),
            "control_id": int(getattr(message, "control_id", 0)),
            "heartbeat": int(getattr(message, "heartbeat", 0)),
            "l2_raw": int(getattr(message, "l2_raw", 0)),
            "r2_raw": int(getattr(message, "r2_raw", 0)),
            "stick_raw": int_list(getattr(message, "stick_raw", [])),
            "buttons": int_list(getattr(message, "buttons", [])),
            "motion_command": int(getattr(message, "motion_command", 0)),
            "skill_command": int(getattr(message, "skill_command", 0)),
            "reserved": int_list(getattr(message, "reserved", [])),
            "normalized_analog": finite_float_list(getattr(message, "normalized_analog", [])),
            "source": "ros2_rc02_diagnostics",
        }
        with self.lock:
            if not self.closed:
                self.latest["rc02_diagnostics"] = record

    def on_policy_diagnostics(self, message: Any) -> None:
        monotonic_ns, ros_time_ns = self.receipt_times()
        record = {
            "receive_monotonic_ns": monotonic_ns,
            "receive_ros_time_ns": ros_time_ns,
            "ros_stamp_ns": message_stamp_ns(message),
            "source_monotonic_ns": int(getattr(message, "source_monotonic_ns", 0)),
            "source_param_tag": str(getattr(message, "source_param_tag", "")),
            "active": bool(getattr(message, "active", False)),
            "policy_output_valid": bool(getattr(message, "policy_output_valid", False)),
            "policy_recurrent": bool(getattr(message, "policy_recurrent", False)),
            "entry_transition_active": bool(getattr(message, "entry_transition_active", False)),
            "entry_transition_elapsed": finite_float(getattr(message, "entry_transition_elapsed", None)),
            "entry_transition_duration": finite_float(getattr(message, "entry_transition_duration", None)),
            "transformed_base_angular_velocity": finite_float_list(
                getattr(message, "transformed_base_angular_velocity", [])),
            "projected_gravity": finite_float_list(getattr(message, "projected_gravity", [])),
            "policy_command": finite_float_list(getattr(message, "policy_command", [])),
            "observation_single": finite_float_list(getattr(message, "observation_single", [])),
            "observation": finite_float_list(getattr(message, "observation", [])),
            "raw_action": finite_float_list(getattr(message, "raw_action", [])),
            "clipped_action": finite_float_list(getattr(message, "clipped_action", [])),
            "controlled_action": finite_float_list(getattr(message, "controlled_action", [])),
            "q_des_policy": finite_float_list(getattr(message, "q_des_policy", [])),
            "q_des_command": finite_float_list(getattr(message, "q_des_command", [])),
            "qd_des_policy": finite_float_list(getattr(message, "qd_des_policy", [])),
            "qd_des_command": finite_float_list(getattr(message, "qd_des_command", [])),
            "feed_forward_torque": finite_float_list(getattr(message, "feed_forward_torque", [])),
            "stiffness": finite_float_list(getattr(message, "stiffness", [])),
            "damping": finite_float_list(getattr(message, "damping", [])),
            "source": "ros2_policy_diagnostics",
        }
        with self.lock:
            if self.closed:
                return
            previous = self.latest.get("policy_diagnostics")
            previous_source_ns = previous.get("source_monotonic_ns") if previous else None
            if previous_source_ns is None or record["source_monotonic_ns"] > previous_source_ns:
                self.latest["policy_diagnostics"] = record

    def on_leo_command_diagnostics(self, message: Any) -> None:
        monotonic_ns, ros_time_ns = self.receipt_times()
        record = leo_command_diagnostics_record(message, monotonic_ns, ros_time_ns)
        with self.lock:
            if self.closed:
                return
            previous = self.latest.get("leo_command_diagnostics")
            previous_source_ns = previous.get("source_monotonic_ns") if previous else None
            if is_new_leo_command_diagnostics(record, previous_source_ns):
                self.latest["leo_command_diagnostics"] = record

    def on_motion_state(self, message: Any) -> None:
        _, ros_time_ns = self.receipt_times()
        name = str(getattr(message, "current_motion_task", "")).strip()
        available = [str(item) for item in getattr(message, "available_transition_motions", [])]
        self.update_motion(name, "ros2", available, ros_time_ns)

    def copy_latest(self, key: str) -> Optional[dict[str, Any]]:
        value = self.latest.get(key)
        return dict(value) if value is not None else None

    def add_command_evidence(self, details: dict[str, Any], sample: dict[str, Any]) -> None:
        """Attach the final joint-command value corresponding to a limit check.

        The command feedback topic is the command after runner-side processing
        (including target clamping and any transition blending).  It is not the
        raw neural-network action, so the event records that provenance
        explicitly.
        """
        target_field_by_event = {
            "model_position_limit_violation": "position",
            "model_position_limit_near": "position",
            "model_velocity_limit_violation": "velocity",
            "model_effort_limit_violation": "torque",
        }
        target_field = target_field_by_event.get(details.get("event_type"))
        index = details.get("joint_index")
        if target_field is None or not isinstance(index, int) or index < 0:
            return

        measured = (sample.get("joint_state") or {}).get(target_field, [])
        if index < len(measured):
            details[f"measured_{target_field}"] = measured[index]

        command = sample.get("joint_command_feedback")
        if not command:
            return
        target = command.get(target_field, [])
        if index >= len(target):
            return

        details[f"target_{target_field}"] = target[index]
        details["target_source"] = "joint_command_feedback"
        details["target_stage"] = "post_runner_processing"
        command_ns = command.get("receive_monotonic_ns")
        sample_ns = sample.get("receive_monotonic_ns")
        if command_ns is not None and sample_ns is not None:
            details["target_age_ms"] = (sample_ns - command_ns) / 1_000_000.0

    def add_posture_evidence(self, details: dict[str, Any], sample: dict[str, Any]) -> None:
        """Attach IMU posture context to limit events during walking motions.

        The SDK does not publish a measured foot-contact topic through this
        interface, so acceleration and angular speed are recorded as motion
        indicators only. They must not be interpreted as proof that a foot is
        airborne.
        """
        if sample.get("motion") not in WALK_MOTIONS:
            return

        imu = sample.get("imu_info")
        if not imu:
            details["posture_context"] = {
                "available": False,
                "reason": "imu_info_unavailable",
                "foot_contact": "not_recorded",
            }
            return

        sample_ns = sample.get("receive_monotonic_ns")
        imu_ns = imu.get("receive_monotonic_ns")
        age_ms = None
        if sample_ns is not None and imu_ns is not None:
            age_ms = (int(sample_ns) - int(imu_ns)) / 1_000_000.0

        tilt_rad = finite_float(imu.get("tilt_rad"))
        angular_speed = finite_float(imu.get("angular_speed_rad_s"))
        acceleration_deviation = finite_float(imu.get("acceleration_deviation_mps2"))
        tilt_threshold = math.radians(self.args.posture_tilt_threshold_deg)
        fallen_threshold = math.radians(self.args.posture_fallen_threshold_deg)
        posture_class = "unknown"
        if tilt_rad is not None:
            if tilt_rad >= fallen_threshold:
                posture_class = "fallen_suspected"
            elif tilt_rad >= tilt_threshold:
                posture_class = "tilted"
            else:
                posture_class = "upright"

        details["posture_context"] = {
            "available": True,
            "fresh": age_ms is None or age_ms <= self.args.imu_max_age_ms,
            "age_ms": age_ms,
            "posture_class": posture_class,
            "tilt_rad": tilt_rad,
            "tilt_deg": imu.get("tilt_deg"),
            "tilt_threshold_deg": self.args.posture_tilt_threshold_deg,
            "fallen_threshold_deg": self.args.posture_fallen_threshold_deg,
            "angular_speed_rad_s": angular_speed,
            "angular_speed_threshold_rad_s": self.args.posture_angular_speed_threshold,
            "acceleration_norm_mps2": imu.get("acceleration_norm_mps2"),
            "acceleration_deviation_mps2": acceleration_deviation,
            "acceleration_deviation_threshold_mps2": self.args.posture_acceleration_deviation,
            "tilt_exceeded": tilt_rad is not None and tilt_rad >= tilt_threshold,
            "fallen_suspected": tilt_rad is not None and tilt_rad >= fallen_threshold,
            "high_angular_speed": (
                angular_speed is not None
                and angular_speed >= self.args.posture_angular_speed_threshold
            ),
            "acceleration_anomaly": (
                acceleration_deviation is not None
                and acceleration_deviation >= self.args.posture_acceleration_deviation
            ),
            "foot_contact": "not_recorded",
            "imu_receive_monotonic_ns": imu.get("receive_monotonic_ns"),
            "imu_receive_ros_time_ns": imu.get("receive_ros_time_ns"),
            "imu_ros_stamp_ns": imu.get("ros_stamp_ns"),
            "rpy": imu.get("rpy"),
            "quaternion_wxyz": imu.get("quaternion_wxyz"),
            "linear_acceleration_mps2": imu.get("linear_acceleration_mps2"),
            "angular_velocity_rad_s": imu.get("angular_velocity_rad_s"),
        }

    def add_leo_command_evidence(self, details: dict[str, Any], sample: dict[str, Any]) -> None:
        diagnostics = sample.get("leo_command_diagnostics")
        policy = sample.get("policy_diagnostics")
        policy_freshness = policy_diagnostics_freshness(
            policy,
            int(sample["receive_monotonic_ns"]),
            self.args.leo_command_max_age_ms,
            sample.get("motion"),
        )
        details["leo_command_diagnostics"] = {
            **(diagnostics or {}),
            "freshness": leo_command_diagnostics_freshness(
                diagnostics,
                int(sample["receive_monotonic_ns"]),
                self.args.leo_command_max_age_ms,
                sample.get("motion"),
            ),
            "command_provenance": "runner_speed_chain_diagnostics",
            "raw_actor_action": "not_recorded",
        }
        details["policy_diagnostics"] = {
            "source_monotonic_ns": policy.get("source_monotonic_ns") if policy else None,
            "source_param_tag": policy.get("source_param_tag") if policy else None,
            "freshness": policy_freshness,
            "policy_output_valid": policy.get("policy_output_valid") if policy else False,
            "raw_action": policy.get("raw_action") if policy_freshness["fresh"] else None,
            "clipped_action": policy.get("clipped_action") if policy_freshness["fresh"] else None,
            "controlled_action": policy.get("controlled_action") if policy_freshness["fresh"] else None,
            "q_des_policy": policy.get("q_des_policy") if policy_freshness["fresh"] else None,
            "q_des_command": policy.get("q_des_command") if policy_freshness["fresh"] else None,
        }
        if policy_freshness["fresh"]:
            details["raw_actor_action"] = policy["raw_action"]
            details["leo_command_diagnostics"]["raw_actor_action"] = policy["raw_action"]
            details["action_provenance"] = "policy_diagnostics.raw_action"
        else:
            details["raw_actor_action"] = "unavailable"
            details["action_provenance"] = f"policy_diagnostics_unavailable:{policy_freshness['reason']}"

    def update_motion(self, name: str, source: str, available: list[str], ros_time_ns: int) -> None:
        if not name:
            return
        monotonic_ns = time.monotonic_ns()
        with self.lock:
            if self.closed:
                return
            old_name = self.motion
            self.motion = name
            self.motion_source = source
            self.motion_time_ns = monotonic_ns
            self.available_transitions = available
            if old_name != name:
                self.emit_instant(
                    "motion_change",
                    {
                        "from_motion": old_name,
                        "to_motion": name,
                        "source": source,
                        "available_transition_motions": available,
                    },
                    self.last_sample,
                    monotonic_ns,
                    ros_time_ns,
                )

    def event_record(
        self,
        event_id: int,
        phase: str,
        event_type: str,
        details: dict[str, Any],
        sample: Optional[dict[str, Any]],
        monotonic_ns: Optional[int] = None,
        ros_time_ns: Optional[int] = None,
    ) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "event_id": event_id,
            "phase": phase,
            "event_type": event_type,
            "receive_monotonic_ns": monotonic_ns if monotonic_ns is not None else time.monotonic_ns(),
            "receive_ros_time_ns": (
                ros_time_ns
                if ros_time_ns is not None
                else int(self.node.get_clock().now().nanoseconds)
            ),
            "sample_index": sample.get("sample_index") if sample else self.sample_count,
            "ros_stamp_ns": sample.get("ros_stamp_ns") if sample else None,
            "motion": sample.get("motion") if sample else self.motion,
            "motion_source": sample.get("motion_source") if sample else self.motion_source,
            **details,
        }

    def emit_instant(
        self,
        event_type: str,
        details: dict[str, Any],
        sample: Optional[dict[str, Any]],
        monotonic_ns: Optional[int] = None,
        ros_time_ns: Optional[int] = None,
    ) -> None:
        event_id = self.next_event_id
        self.next_event_id += 1
        self.event_record_count += 1
        self.write_line(
            self.events_file,
            self.event_record(event_id, "instant", event_type, details, sample, monotonic_ns, ros_time_ns),
            force_sync=True,
        )

    def start_condition(self, key: str, details: dict[str, Any], sample: dict[str, Any]) -> None:
        event_id = self.next_event_id
        self.next_event_id += 1
        initial_excess = finite_float(details.get("excess"))
        self.active[key] = {
            "event_id": event_id,
            "event_type": details["event_type"],
            "started_monotonic_ns": sample["receive_monotonic_ns"],
            "started_ros_time_ns": sample["receive_ros_time_ns"],
            "started_sample_index": sample["sample_index"],
            "motion_start": sample.get("motion"),
            "last_details": dict(details),
            "max_excess": initial_excess,
            "peak_details": dict(details) if initial_excess is not None else None,
            "peak_sample_index": sample["sample_index"] if initial_excess is not None else None,
            "peak_receive_monotonic_ns": sample["receive_monotonic_ns"] if initial_excess is not None else None,
            "peak_receive_ros_time_ns": sample["receive_ros_time_ns"] if initial_excess is not None else None,
        }
        self.event_record_count += 1
        self.write_line(
            self.events_file,
            self.event_record(event_id, "start", details["event_type"], details, sample),
            force_sync=True,
        )

    def end_condition(
        self,
        active: dict[str, Any],
        sample: Optional[dict[str, Any]],
        close_reason: str,
    ) -> None:
        current_sample = sample or self.last_sample
        details = dict(active["last_details"])
        details.pop("event_type", None)
        end_ns = (
            int(current_sample["receive_monotonic_ns"])
            if current_sample
            else time.monotonic_ns()
        )
        details.update(
            {
                "started_monotonic_ns": active["started_monotonic_ns"],
                "started_ros_time_ns": active["started_ros_time_ns"],
                "started_sample_index": active["started_sample_index"],
                "duration_ms": (end_ns - active["started_monotonic_ns"]) / 1_000_000.0,
                "motion_start": active["motion_start"],
                "motion_end": current_sample.get("motion") if current_sample else self.motion,
                "close_reason": close_reason,
            }
        )
        if active.get("max_excess") is not None:
            details["max_excess"] = active["max_excess"]
        if active.get("peak_details") is not None:
            details["peak"] = {
                **active["peak_details"],
                "sample_index": active.get("peak_sample_index"),
                "receive_monotonic_ns": active.get("peak_receive_monotonic_ns"),
                "receive_ros_time_ns": active.get("peak_receive_ros_time_ns"),
            }
        self.event_record_count += 1
        self.write_line(
            self.events_file,
            self.event_record(
                active["event_id"],
                "end",
                active["event_type"],
                details,
                current_sample,
                monotonic_ns=end_ns,
            ),
            force_sync=True,
        )

    def sync_condition(self, key: str, details: dict[str, Any], sample: dict[str, Any]) -> None:
        active = self.active.get(key)
        if active is None:
            self.start_condition(key, details, sample)
            return
        active["last_details"] = dict(details)
        excess = finite_float(details.get("excess"))
        if excess is not None:
            current_max = active.get("max_excess")
            if current_max is None or excess > current_max:
                active["max_excess"] = excess
                active["peak_details"] = dict(details)
                active["peak_sample_index"] = sample["sample_index"]
                active["peak_receive_monotonic_ns"] = sample["receive_monotonic_ns"]
                active["peak_receive_ros_time_ns"] = sample["receive_ros_time_ns"]

    def evaluate(self, sample: dict[str, Any]) -> None:
        conditions: dict[str, dict[str, Any]] = {}
        state = sample["joint_state"]
        positions = state["position"]
        velocities = state["velocity"]
        torques = state["torque"]

        for index, name in enumerate(self.joint_names):
            position = positions[index] if index < len(positions) else None
            velocity = velocities[index] if index < len(velocities) else None
            torque = torques[index] if index < len(torques) else None
            for field, value in (("position", position), ("velocity", velocity), ("torque", torque)):
                if value is None:
                    conditions[f"non_finite:{field}:{index}"] = {
                        "event_type": "non_finite_joint_state",
                        "joint_index": index,
                        "joint_name": name,
                        "field": field,
                    }

            limit = self.limits.get(name, {})
            lower, upper = limit.get("lower"), limit.get("upper")
            if position is not None and lower is not None and upper is not None:
                if position < lower:
                    conditions[f"position_violation:{index}:lower"] = {
                        "event_type": "model_position_limit_violation",
                        "joint_index": index,
                        "joint_name": name,
                        "side": "lower",
                        "value": position,
                        "limit": lower,
                        "excess": lower - position,
                        "unit": "rad",
                    }
                elif position > upper:
                    conditions[f"position_violation:{index}:upper"] = {
                        "event_type": "model_position_limit_violation",
                        "joint_index": index,
                        "joint_name": name,
                        "side": "upper",
                        "value": position,
                        "limit": upper,
                        "excess": position - upper,
                        "unit": "rad",
                    }
                elif self.args.position_margin > 0.0:
                    if position - lower <= self.args.position_margin:
                        conditions[f"position_near_limit:{index}:lower"] = {
                            "event_type": "model_position_limit_near",
                            "joint_index": index,
                            "joint_name": name,
                            "side": "lower",
                            "value": position,
                            "limit": lower,
                            "distance": position - lower,
                            "margin": self.args.position_margin,
                            "unit": "rad",
                        }
                    if upper - position <= self.args.position_margin:
                        conditions[f"position_near_limit:{index}:upper"] = {
                            "event_type": "model_position_limit_near",
                            "joint_index": index,
                            "joint_name": name,
                            "side": "upper",
                            "value": position,
                            "limit": upper,
                            "distance": upper - position,
                            "margin": self.args.position_margin,
                            "unit": "rad",
                        }

            velocity_limit = limit.get("velocity")
            if velocity is not None and velocity_limit is not None and abs(velocity) > velocity_limit:
                conditions[f"velocity_violation:{index}"] = {
                    "event_type": "model_velocity_limit_violation",
                    "joint_index": index,
                    "joint_name": name,
                    "value": velocity,
                    "limit": velocity_limit,
                    "excess": abs(velocity) - velocity_limit,
                    "unit": "rad/s",
                }
            effort_limit = limit.get("effort")
            if torque is not None and effort_limit is not None and abs(torque) > effort_limit:
                conditions[f"effort_violation:{index}"] = {
                    "event_type": "model_effort_limit_violation",
                    "joint_index": index,
                    "joint_name": name,
                    "value": torque,
                    "limit": effort_limit,
                    "excess": abs(torque) - effort_limit,
                    "unit": "N*m",
                }

        debug = sample.get("motor_debug") or {}
        for index, code in enumerate(debug.get("error_code", [])):
            if code not in (None, 0):
                conditions[f"motor_error:{index}:{code}"] = {
                    "event_type": "motor_error_code",
                    "motor_index": index,
                    "code": code,
                }
        for index, value in enumerate(debug.get("offline", [])):
            if value not in (None, 0):
                conditions[f"motor_offline:{index}"] = {
                    "event_type": "motor_offline",
                    "motor_index": index,
                    "value": value,
                }
        for index, value in enumerate(debug.get("enable", [])):
            if value == 0:
                conditions[f"motor_disabled:{index}"] = {
                    "event_type": "motor_disabled",
                    "motor_index": index,
                    "value": value,
                }

        power = sample.get("power_info")
        if power and power.get("error_code") not in (None, 0):
            conditions["power_error"] = {
                "event_type": "power_error_code",
                "code": power["error_code"],
            }

        threshold = self.args.tracking_error_threshold
        command = sample.get("joint_command_feedback")
        if threshold > 0.0 and command:
            commanded_positions = command.get("position", [])
            for index, position in enumerate(positions):
                commanded = commanded_positions[index] if index < len(commanded_positions) else None
                if position is not None and commanded is not None:
                    error = commanded - position
                    if abs(error) > threshold:
                        conditions[f"command_gap:{index}"] = {
                            "event_type": "command_state_gap",
                            "joint_index": index,
                            "joint_name": self.joint_names[index],
                            "commanded_position": commanded,
                            "measured_position": position,
                            "value": error,
                            "limit": threshold,
                            "excess": abs(error) - threshold,
                            "unit": "rad",
                        }

        for details in conditions.values():
            self.add_command_evidence(details, sample)
            self.add_leo_command_evidence(details, sample)
            if details.get("event_type") in POSTURE_EVENT_TYPES:
                self.add_posture_evidence(details, sample)
        for key, details in conditions.items():
            self.sync_condition(key, details, sample)
        for key in list(self.active):
            if key not in conditions:
                self.end_condition(self.active.pop(key), sample, "condition_cleared")

    def start_lcm(self) -> None:
        try:
            import lcm

            self.lcm = lcm.LCM(self.args.lcm_url)
            self.lcm.subscribe(self.args.lcm_channel, self.on_lcm_task_state)
            if not self.args.no_sim_state:
                self.lcm.subscribe(self.args.lcm_sim_state_channel, self.on_lcm_sim_state)
            if not self.args.no_control_diagnostics:
                self.lcm.subscribe(self.args.lcm_control_diagnostics_channel, self.on_lcm_control_diagnostics)
            self.lcm_thread = threading.Thread(target=self.lcm_loop, name="safety-lcm", daemon=True)
            self.lcm_thread.start()
        except Exception as error:
            self.lcm_error = str(error)
            self.node.get_logger().warning(f"LCM task-state recorder disabled: {error}")

    def lcm_loop(self) -> None:
        while not self.lcm_stop.is_set() and self.lcm is not None:
            try:
                self.lcm.handle_timeout(100)
            except Exception as error:
                self.lcm_error = str(error)
                self.node.get_logger().warning(f"LCM task-state handler stopped: {error}")
                return

    def on_lcm_task_state(self, channel: str, payload: bytes) -> None:
        del channel
        name = decode_lcm_task_state(payload)
        if name is not None:
            self.update_motion(name, "lcm", [], int(self.node.get_clock().now().nanoseconds))

    def on_lcm_sim_state(self, channel: str, payload: bytes) -> None:
        del channel
        record = decode_lcm_sim_state(payload)
        if record is not None:
            with self.lock:
                if not self.closed:
                    record["receive_ros_time_ns"] = int(self.node.get_clock().now().nanoseconds)
                    self.latest["sim_state"] = record

    def on_lcm_control_diagnostics(self, channel: str, payload: bytes) -> None:
        del channel
        record = decode_lcm_dynamic_vector(payload)
        if record is not None:
            with self.lock:
                if not self.closed:
                    record["receive_ros_time_ns"] = int(self.node.get_clock().now().nanoseconds)
                    self.latest["sim_control_diagnostics"] = record

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self.lcm_stop.set()
        if self.lcm_thread is not None:
            self.lcm_thread.join(timeout=1.0)
        with self.lock:
            for key in list(self.active):
                self.end_condition(self.active.pop(key), self.last_sample, "recorder_stopped")
            self.metadata.update(
                {
                    "finished_at_utc": utc_now(),
                    "sample_count": self.sample_count,
                    "event_record_count": self.event_record_count,
                    "final_motion": self.motion,
                    "final_motion_source": self.motion_source,
                    "lcm_error": self.lcm_error,
                }
            )
            self.write_metadata()
            self.samples_file.flush()
            self.events_file.flush()
            os.fsync(self.samples_file.fileno())
            os.fsync(self.events_file.fileno())
            self.samples_file.close()
            self.events_file.close()


def positive_float(value: str) -> float:
    result = float(value)
    if result <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return result


def non_negative_float(value: str) -> float:
    result = float(value)
    if result < 0.0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return result


def build_parser(repo_root: Path) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record read-only T800 safety telemetry from existing ROS2/LCM interfaces."
    )
    parser.add_argument(
        "--output",
        help="run directory under <repo>/records/safety_monitor; default is t800_<timestamp>",
    )
    parser.add_argument(
        "--model-yaml",
        default=str(repo_root / "assets/config/t800/model/default.yaml"),
        help="model YAML used for joint order",
    )
    parser.add_argument(
        "--urdf",
        default=str(repo_root / "assets/resource/robot/t800/urdf/serial_t800.urdf"),
        help="URDF used for model limit comparison",
    )
    parser.add_argument("--config", action="append", default=[], help="config path recorded in metadata")
    parser.add_argument(
        "--lcm-config",
        default=str(repo_root / "assets/config/t800/lcm/default.yaml"),
    )
    parser.add_argument("--lcm-url", default=None)
    parser.add_argument("--lcm-channel", default="task_state")
    parser.add_argument("--lcm-sim-state-channel", default="sim_state")
    parser.add_argument("--no-sim-state", action="store_true")
    parser.add_argument("--lcm-control-diagnostics-channel", default="sim_control_diagnostics")
    parser.add_argument("--no-control-diagnostics", action="store_true")
    parser.add_argument("--no-lcm", action="store_true")
    parser.add_argument("--joint-state-topic", default="/hardware/joint_state")
    parser.add_argument("--motor-state-topic", default="/hardware/motor_state")
    parser.add_argument("--joint-command-topic", default="/hardware/joint_command_feedback")
    parser.add_argument("--motor-command-topic", default="/hardware/motor_command")
    parser.add_argument("--motor-debug-topic", default="/hardware/motor_debug")
    parser.add_argument("--power-info-topic", default="/hardware/power_info")
    parser.add_argument("--imu-topic", default="/hardware/imu_info")
    parser.add_argument("--gamepad-keys-topic", default="/hardware/gamepad_keys")
    parser.add_argument("--rc02-diagnostics-topic", default="/hardware/rc02_diagnostics")
    parser.add_argument("--policy-diagnostics-topic", default="/motion/policy_diagnostics")
    parser.add_argument("--motion-topic", default="/motion/motion_state")
    parser.add_argument(
        "--leo-command-diagnostics-topic", default="/motion/leo_command_diagnostics"
    )
    parser.add_argument(
        "--leo-command-max-age-ms",
        type=positive_float,
        default=100.0,
        help="source plus receipt age limit; current evidence also requires active/matching Leo motion",
    )
    parser.add_argument("--duration", type=positive_float, default=None)
    parser.add_argument("--position-margin", type=non_negative_float, default=0.02)
    parser.add_argument("--tracking-error-threshold", type=non_negative_float, default=0.0)
    parser.add_argument("--expected-joint-rate", type=positive_float, default=500.0)
    parser.add_argument("--gap-factor", type=positive_float, default=5.0)
    parser.add_argument(
        "--imu-max-age-ms",
        type=positive_float,
        default=20.0,
        help="maximum age for a fresh IMU posture context",
    )
    parser.add_argument(
        "--posture-tilt-threshold-deg",
        type=positive_float,
        default=35.0,
        help="tilt threshold for the tilted posture flag",
    )
    parser.add_argument(
        "--posture-fallen-threshold-deg",
        type=positive_float,
        default=60.0,
        help="tilt threshold for fallen_suspected",
    )
    parser.add_argument(
        "--posture-angular-speed-threshold",
        type=positive_float,
        default=6.0,
        help="angular-speed threshold for the high-angular-speed flag",
    )
    parser.add_argument(
        "--posture-acceleration-deviation",
        type=positive_float,
        default=3.0,
        help="acceleration deviation from 1 g in m/s^2 for the anomaly flag",
    )
    parser.add_argument("--fsync-every", type=int, default=100)
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main() -> int:
    repo_root = Path(__file__).resolve().parents[2]
    args = build_parser(repo_root).parse_args()
    args.repo_root = repo_root
    args.model_yaml = resolve_path(args.model_yaml, repo_root)
    args.urdf = resolve_path(args.urdf, repo_root)
    args.lcm_config = resolve_path(args.lcm_config, repo_root)
    args.config = [str(resolve_path(path, repo_root)) for path in args.config]
    args.lcm_url = args.lcm_url or lcm_url_from_config(args.lcm_config)
    if args.posture_fallen_threshold_deg <= args.posture_tilt_threshold_deg:
        print(
            "configuration error: --posture-fallen-threshold-deg must be greater than "
            "--posture-tilt-threshold-deg",
            file=sys.stderr,
        )
        return 2
    records_root = (repo_root / "records" / "safety_monitor").resolve()
    if args.output:
        output_path = Path(args.output).expanduser()
        args.output = str((records_root / output_path if not output_path.is_absolute() else output_path).resolve())
    else:
        stamp = datetime_lib.datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = str((records_root / f"t800_{stamp}").resolve())
    output_path = Path(args.output)
    try:
        output_path.relative_to(records_root)
    except ValueError:
        print(
            "configuration error: output must stay inside repository records/safety_monitor: "
            f"{output_path}",
            file=sys.stderr,
        )
        return 2
    if Path(args.output).exists():
        print(f"output directory already exists: {args.output}", file=sys.stderr)
        return 2

    try:
        joint_names = load_joint_names(args.model_yaml)
        limits = load_urdf_limits(args.urdf)
    except (OSError, ValueError, element_tree.ParseError, yaml.YAMLError) as error:
        print(f"configuration error: {error}", file=sys.stderr)
        return 2

    missing = [name for name in joint_names if name not in limits]
    if missing:
        print("warning: URDF has no limits for: " + ", ".join(missing), file=sys.stderr)
    if args.dry_run:
        print(
            json.dumps(
                {
                    "output": args.output,
                    "model_yaml": str(args.model_yaml),
                    "urdf": str(args.urdf),
                    "joint_count": len(joint_names),
                    "joint_names": joint_names,
                    "limits_found": len(limits),
                    "missing_limits": missing,
                    "lcm_url": args.lcm_url,
                    "imu_topic": args.imu_topic,
                    "posture_thresholds": {
                        "imu_max_age_ms": args.imu_max_age_ms,
                        "tilt_deg": args.posture_tilt_threshold_deg,
                        "fallen_suspected_deg": args.posture_fallen_threshold_deg,
                        "angular_speed_rad_s": args.posture_angular_speed_threshold,
                        "acceleration_deviation_mps2": args.posture_acceleration_deviation,
                    },
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0

    try:
        import rclpy
        from interface_protocol.msg import GamepadKeys, ImuInfo, JointCommand, JointState, MotorDebug, PowerInfo
        try:
            from interface_protocol.msg import MotionState
        except ImportError:
            MotionState = None
        try:
            from interface_protocol.msg import LeoCommandDiagnostics
        except ImportError:
            LeoCommandDiagnostics = None
        try:
            from interface_protocol.msg import PolicyDiagnostics
        except ImportError:
            PolicyDiagnostics = None
        try:
            from interface_protocol.msg import Rc02Diagnostics
        except ImportError:
            Rc02Diagnostics = None
        from rclpy.node import Node
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    except ImportError as error:
        print(
            "ROS2 message imports failed. Source /opt/ros/humble and the SDK ROS2 overlay first: "
            f"{error}",
            file=sys.stderr,
        )
        return 2

    qos_profile = QoSProfile(depth=20)
    qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
    qos_profile.durability = QoSDurabilityPolicy.VOLATILE
    rclpy.init()
    node = Node("t800_safety_recorder")
    recorder: Optional[SafetyRecorder] = None
    try:
        recorder = SafetyRecorder(
            node,
            args,
            (
                JointState,
                JointCommand,
                MotorDebug,
                PowerInfo,
                MotionState,
                ImuInfo,
                LeoCommandDiagnostics,
                GamepadKeys,
                Rc02Diagnostics,
                PolicyDiagnostics,
            ),
            joint_names,
            limits,
            qos_profile,
        )
        node.get_logger().info(f"Read-only safety recorder writing to {args.output}")
        node.get_logger().info(
            f"Monitoring {len(joint_names)} joints; model limits found for {len(limits)} joints"
        )
        if args.duration is None:
            rclpy.spin(node)
        else:
            while rclpy.ok():
                remaining_ns = recorder.duration_deadline_ns - time.monotonic_ns()
                if remaining_ns <= 0:
                    node.get_logger().info("Safety recording duration reached; stopping.")
                    break
                rclpy.spin_once(node, timeout_sec=min(0.25, remaining_ns / 1_000_000_000.0))
    except KeyboardInterrupt:
        node.get_logger().info("Stopping safety recorder on Ctrl-C.")
    finally:
        if recorder is not None:
            recorder.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"Safety log written to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

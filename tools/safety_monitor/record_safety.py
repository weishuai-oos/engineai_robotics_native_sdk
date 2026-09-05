#!/usr/bin/env python3
"""Read-only T800 safety telemetry recorder.

This process subscribes to existing ROS2 topics and the optional LCM task_state
channel. It never publishes commands and never changes robot state.
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
DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=0"


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


class SafetyRecorder:
    def __init__(
        self,
        node: Any,
        args: argparse.Namespace,
        message_types: tuple[Any, Any, Any, Any, Any],
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
            "schema_version": "t800_safety_log_v1",
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
            },
            "lcm": {
                "enabled": not args.no_lcm,
                "url": args.lcm_url,
                "channel": args.lcm_channel,
            },
            "thresholds": {
                "position_margin_rad": args.position_margin,
                "tracking_error_rad": args.tracking_error_threshold,
                "expected_joint_rate_hz": args.expected_joint_rate,
                "gap_factor": args.gap_factor,
            },
            "files": {
                "samples": str(self.samples_path),
                "events": str(self.events_path),
            },
        }
        self.write_metadata()

        joint_state_type, joint_command_type, motor_debug_type, power_info_type, motion_state_type = message_types
        self.subscriptions = [
            node.create_subscription(joint_state_type, args.joint_state_topic, self.on_joint_state, qos_profile),
            node.create_subscription(joint_state_type, args.motor_state_topic, self.on_motor_state, qos_profile),
            node.create_subscription(joint_command_type, args.joint_command_topic, self.on_joint_command, qos_profile),
            node.create_subscription(joint_command_type, args.motor_command_topic, self.on_motor_command, qos_profile),
            node.create_subscription(motor_debug_type, args.motor_debug_topic, self.on_motor_debug, qos_profile),
            node.create_subscription(power_info_type, args.power_info_topic, self.on_power_info, qos_profile),
        ]
        if motion_state_type is not None:
            self.subscriptions.append(
                node.create_subscription(motion_state_type, args.motion_topic, self.on_motion_state, qos_profile)
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
            sample = {
                "schema_version": 1,
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
    parser.add_argument("--output", help="new output directory; default is safety_logs/t800_<timestamp>")
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
    parser.add_argument("--no-lcm", action="store_true")
    parser.add_argument("--joint-state-topic", default="/hardware/joint_state")
    parser.add_argument("--motor-state-topic", default="/hardware/motor_state")
    parser.add_argument("--joint-command-topic", default="/hardware/joint_command_feedback")
    parser.add_argument("--motor-command-topic", default="/hardware/motor_command")
    parser.add_argument("--motor-debug-topic", default="/hardware/motor_debug")
    parser.add_argument("--power-info-topic", default="/hardware/power_info")
    parser.add_argument("--motion-topic", default="/motion/motion_state")
    parser.add_argument("--duration", type=positive_float, default=None)
    parser.add_argument("--position-margin", type=non_negative_float, default=0.02)
    parser.add_argument("--tracking-error-threshold", type=non_negative_float, default=0.0)
    parser.add_argument("--expected-joint-rate", type=positive_float, default=500.0)
    parser.add_argument("--gap-factor", type=positive_float, default=5.0)
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
    if args.output:
        output_path = Path(args.output).expanduser()
        args.output = str((Path.cwd() / output_path if not output_path.is_absolute() else output_path).resolve())
    else:
        stamp = datetime_lib.datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = str((repo_root / "safety_logs" / f"t800_{stamp}").resolve())
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
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0

    try:
        import rclpy
        from interface_protocol.msg import JointCommand, JointState, MotorDebug, PowerInfo
        try:
            from interface_protocol.msg import MotionState
        except ImportError:
            MotionState = None
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
            (JointState, JointCommand, MotorDebug, PowerInfo, MotionState),
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

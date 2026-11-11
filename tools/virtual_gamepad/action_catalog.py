"""Numbered policy catalog and commands shared by the launcher and Qt UI."""

import json
from pathlib import Path
import re
import struct


CAPACITY = 50
READY_MOTIONS = frozenset({
    "pd_stand", "pd_stand_x", "pd_stand_y",
    "walk", "walk_impact", "walk_leo", "walk_leo_terrain",
})
BUTTON_NAMES = ("LB", "RB", "A", "B", "X", "Y")
MARKER_KEYS = ("CROSS_X_UP", "CROSS_X_DOWN", "CROSS_Y_LEFT", "CROSS_Y_RIGHT")


def motion_name(number):
    if type(number) is not int or not 1 <= number <= CAPACITY:
        raise ValueError("Action number must be an integer between 1 and 50")
    return f"test_action_{number:02d}"


def action_keys(number):
    """Use the existing LT/RT markers plus six bits of action number."""
    motion_name(number)
    return ("LT", "RT", *(name for index, name in enumerate(BUTTON_NAMES)
                           if number & (1 << index)))


def task_keys(number):
    return [*MARKER_KEYS, *action_keys(number)[2:]]


def load_catalog(path, check_assets=False):
    path = Path(path)
    catalog = json.loads(path.read_text(encoding="utf-8"))
    if (not isinstance(catalog, dict)
            or catalog.get("schema_version") != 1 or catalog.get("capacity") != CAPACITY
            or not isinstance(catalog.get("actions"), list)):
        raise ValueError("Unsupported numbered action catalog")
    numbers = set()
    for action in catalog["actions"]:
        if not isinstance(action, dict):
            raise ValueError("Catalog actions must be objects")
        number = action.get("id")
        motion_name(number)
        if number in numbers:
            raise ValueError(f"Duplicate action number: {number}")
        numbers.add(number)
        if (not isinstance(action.get("label"), str) or not action["label"].strip()
                or type(action.get("batch")) is not int or action["batch"] < 1
                or type(action.get("iteration")) is not int or action["iteration"] < 0
                or not isinstance(action.get("package"), str)
                or not re.fullmatch(r"rl_supermimic_[A-Za-z0-9_]+", action["package"])):
            raise ValueError(f"Invalid catalog entry: {number}")
        if check_assets:
            package = path.parent / action["package"]
            for relative in ("default.yaml", "policies/policy.mnn",
                             "trajectories/supermimic_reference.npz"):
                if not (package / relative).is_file():
                    raise ValueError(f"Missing numbered action asset: {package / relative}")
    catalog["actions"].sort(key=lambda action: action["id"])
    return catalog


def decode_task_state(payload):
    """Read the SDK's existing data::TaskState LCM string (no new protocol)."""
    # Fingerprint from core/include/data/lcm_data/TaskState.hpp.
    if len(payload) < 13:
        raise ValueError("Truncated task_state")
    fingerprint, size = struct.unpack(">QI", payload[:12])
    if (fingerprint != 0x104561AA026A2B0E or size < 1
            or size > 1024 or len(payload) != 12 + size or payload[-1] != 0):
        raise ValueError("Invalid task_state")
    return payload[12:-1].decode("utf-8")

"""Protocol and input validation for numbered actions; no GUI dependencies."""

import json
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

from tools.virtual_gamepad.action_catalog import (
    CAPACITY, READY_MOTIONS, action_keys, decode_task_state, load_catalog, motion_name, task_keys,
)


class ActionCatalogTest(unittest.TestCase):
    def test_shared_example_is_an_empty_catalog(self):
        example = Path(__file__).with_name("catalog.example.json")
        self.assertEqual(load_catalog(example, check_assets=True),
                         {"schema_version": 1, "capacity": 50, "actions": []})

    def test_git_includes_shared_test_policies_and_runner_document(self):
        repo = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / ".gitignore").write_bytes((repo / ".gitignore").read_bytes())
            numbered = "assets/config/t800/numbered_actions/"
            shared = [numbered + "catalog.json", numbered + "rl_supermimic_test/default.yaml",
                      numbered + "rl_supermimic_test/policies/policy.mnn",
                      numbered + "rl_supermimic_test/trajectories/supermimic_reference.npz",
                      "tools/virtual_gamepad/catalog.example.json",
                      "assets/config/t800/rl_supermimic_formal/policies/policy.mnn",
                      "docs/t800_runner_classification_and_transitions.md"]
            ignored = [numbered + "__pycache__/catalog.pyc", "docs/local_notes.md"]
            for relative in shared + ignored:
                path = root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b"fixture")
            subprocess.run(["git", "init", "--quiet"], cwd=root, check=True,
                           capture_output=True)
            for relative in shared:
                result = subprocess.run(["git", "check-ignore", relative], cwd=root,
                                        text=True, capture_output=True)
                self.assertEqual(result.returncode, 1, relative)
            for relative in ignored:
                result = subprocess.run(["git", "check-ignore", relative], cwd=root,
                                        text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, relative)
            result = subprocess.run(["git", "add", "--dry-run", "."], cwd=root,
                                    text=True, capture_output=True, check=True)
            for relative in shared:
                self.assertIn(relative, result.stdout)
            for relative in ignored:
                self.assertNotIn(relative, result.stdout)

    def test_impact_gait_is_ready_for_numbered_actions(self):
        self.assertIn("walk_impact", READY_MOTIONS)
        self.assertNotIn("walk_custom", READY_MOTIONS)

    def test_fifty_unique_commands_use_both_trigger_markers(self):
        names = ("LB", "RB", "A", "B", "X", "Y", "BACK", "START",
                 "CROSS_X_UP", "CROSS_X_DOWN", "CROSS_Y_LEFT", "CROSS_Y_RIGHT")
        encoded = []
        for number in range(1, CAPACITY + 1):
            keys = action_keys(number)
            self.assertEqual(keys[:2], ("LT", "RT"))
            mask = sum(1 << names.index(key) for key in task_keys(number))
            self.assertEqual(mask, 0xF00 | number)
            encoded.append(mask)
            self.assertEqual(motion_name(number), f"test_action_{number:02d}")
        self.assertEqual(len(set(encoded)), CAPACITY)
        for invalid in (0, 51, -1, 1.5, "1", True, None):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                action_keys(invalid)

    def test_invalid_catalogs_are_rejected_and_assets_checked(self):
        entry = {"id": 1, "label": "左勾拳", "batch": 1, "iteration": 74000,
                 "package": "rl_supermimic_left_hook"}
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "catalog.json"
            valid = {"schema_version": 1, "capacity": 50, "actions": [entry]}
            for data in ([], {**valid, "capacity": 49}, {**valid, "actions": [None]},
                         {**valid, "actions": [entry, entry]},
                         {**valid, "actions": [{**entry, "id": 51}]},
                         {**valid, "actions": [{**entry, "package": "../outside"}]}):
                path.write_text(json.dumps(data))
                with self.subTest(data=data), self.assertRaises(ValueError):
                    load_catalog(path)
            path.write_text(json.dumps(valid))
            self.assertEqual(load_catalog(path)["actions"], [entry])
            with self.assertRaisesRegex(ValueError, "Missing numbered action asset"):
                load_catalog(path, check_assets=True)
            for relative in ("default.yaml", "policies/policy.mnn",
                             "trajectories/supermimic_reference.npz"):
                asset = path.parent / entry["package"] / relative
                asset.parent.mkdir(parents=True, exist_ok=True)
                asset.write_bytes(b"runtime asset")
            self.assertEqual(load_catalog(path, check_assets=True)["actions"], [entry])

    def test_task_state_wire_format_and_malformed_packets(self):
        motion = b"test_action_27\x00"
        packet = struct.pack(">QI", 0x104561AA026A2B0E, len(motion)) + motion
        self.assertEqual(decode_task_state(packet), "test_action_27")
        for invalid in (b"", packet[:10], packet[:-1], packet + b"x",
                        b"\xff" * 8 + packet[8:], packet[:-1] + b"x",
                        struct.pack(">QI", 0x104561AA026A2B0E, 0)):
            with self.subTest(packet=invalid), self.assertRaises(ValueError):
                decode_task_state(invalid)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""Schema and provenance checks for the canonical SDK T800 contract."""

import hashlib
import json
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / "assets/config/t800/safety/t800_safety_contract.json"


class T800SafetyContractV2Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = json.loads(CONTRACT_PATH.read_text())
        urdf_path = ROOT / cls.contract["provenance"]["source_urdf"]
        cls.urdf = {
            joint.attrib["name"]: joint
            for joint in ET.parse(urdf_path).getroot().findall("joint")
            if joint.attrib.get("type") != "fixed"
        }

    def test_schema_is_minimal_and_complete(self):
        c = self.contract
        self.assertEqual(c["schema_version"], 2)
        self.assertEqual(c["qualification"], "urdf_derived_provisional")
        self.assertEqual(
            set(c["recovery"]),
            {"position_overrun_rad", "max_outward_velocity_rad_s", "healthy_frames", "note"},
        )
        self.assertEqual(c["recovery"]["position_overrun_rad"], 0.01)
        self.assertEqual(c["recovery"]["max_outward_velocity_rad_s"], 0.2)
        self.assertEqual(c["recovery"]["healthy_frames"], 50)
        self.assertTrue(c["recovery"]["note"])
        self.assertEqual(len(c["joints"]), 25)
        self.assertNotIn("policy_control_period_s", c)
        self.assertNotIn("collision", c)
        self.assertEqual(len({j["sdk_index"] for j in c["joints"]}), 25)
        for joint in c["joints"]:
            self.assertNotIn("policy_index", joint)
            self.assertNotIn("host_name", joint)
            self.assertNotIn("host_index", joint)
            self.assertNotIn("host_dfs_index", joint)
            self.assertNotIn("canonical_name", joint)
            self.assertEqual(set(joint), {"sdk_name", "sdk_index", "hard_limits",
                                          "operational_limits"} | ({"override"} if "override" in joint else set()))

    def test_urdf_provenance_and_default_scaling(self):
        c = self.contract
        urdf_path = ROOT / c["provenance"]["source_urdf"]
        digest = hashlib.sha256(urdf_path.read_bytes()).hexdigest()
        self.assertEqual(digest, c["provenance"]["source_urdf_sha256"])
        scale = c["provenance"]["default_scale"]
        self.assertEqual(scale, {"position": 0.9, "velocity": 0.9, "effort": 0.9})
        for joint in c["joints"]:
            name = joint["sdk_name"]
            limit = self.urdf[name].find("limit")
            hard = joint["hard_limits"]
            op = joint["operational_limits"]
            urdf = {"position_lower_rad": float(limit.attrib["lower"]),
                    "position_upper_rad": float(limit.attrib["upper"]),
                    "velocity_rad_s": float(limit.attrib["velocity"]),
                    "effort_nm": float(limit.attrib["effort"])}
            self.assertEqual(hard, urdf)
            self.assertTrue(all(math.isfinite(v) and v > 0 for k, v in hard.items() if k in ("velocity_rad_s", "effort_nm")))
            self.assertLessEqual(op["velocity_rad_s"], hard["velocity_rad_s"])
            self.assertLessEqual(op["effort_nm"], hard["effort_nm"])
            self.assertAlmostEqual(op["velocity_rad_s"], 0.9 * hard["velocity_rad_s"])
            self.assertAlmostEqual(op["effort_nm"], 0.9 * hard["effort_nm"])
            expected_lower = 0.9 * hard["position_lower_rad"]
            expected_upper = 0.9 * hard["position_upper_rad"]
            if "override" in joint:
                override = joint["override"]
                self.assertTrue(override["reason"] and override["evidence"])
                self.assertGreaterEqual(op["position_lower_rad"], expected_lower)
                self.assertLessEqual(op["position_upper_rad"], expected_upper)
            else:
                self.assertAlmostEqual(op["position_lower_rad"], expected_lower)
                self.assertAlmostEqual(op["position_upper_rad"], expected_upper)
            self.assertGreater(hard["position_upper_rad"], op["position_upper_rad"])

    def test_signed_effort_envelope(self):
        for joint in self.contract["joints"]:
            effort = joint["operational_limits"]["effort_nm"]
            self.assertGreater(effort, 0)
            self.assertLessEqual(-effort, effort)


if __name__ == "__main__":
    unittest.main()

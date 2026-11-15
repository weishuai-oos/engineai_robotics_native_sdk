import sys
import struct
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parent))
import record_safety


class LeoDiagnosticsTest(unittest.TestCase):
    def message(self, source_ns=1234):
        return SimpleNamespace(
            source_monotonic_ns=source_ns,
            source_param_tag="rl_walking_leolab_example",
            active=True,
            raw_stick=[1.0, 2.0, 3.0],
            target_tactical=[4.0, 5.0, 6.0],
            shaped_tactical=[7.0, 8.0, 9.0],
            policy_command=[10.0, 11.0, 12.0],
            source_age_ms=0.25,
        )

    def test_record_preserves_speed_chain_and_provenance(self):
        record = record_safety.leo_command_diagnostics_record(self.message(), 2_000, 3_000)
        self.assertEqual(record["source_param_tag"], "rl_walking_leolab_example")
        self.assertEqual(record["raw_stick"], [1.0, 2.0, 3.0])
        self.assertEqual(record["policy_command"], [10.0, 11.0, 12.0])
        self.assertEqual(record["source_age_ms"], 0.25)

    def test_duplicate_or_backward_source_sample_is_rejected(self):
        record = record_safety.leo_command_diagnostics_record(self.message(100), 2_000, 3_000)
        self.assertFalse(record_safety.is_new_leo_command_diagnostics(record, 100))
        self.assertFalse(record_safety.is_new_leo_command_diagnostics(record, 101))
        self.assertTrue(record_safety.is_new_leo_command_diagnostics(record, None))

    def test_freshness_uses_local_receipt_clock_not_remote_source_clock(self):
        record = record_safety.leo_command_diagnostics_record(self.message(1), 1_000_000, 0)
        freshness = record_safety.leo_command_diagnostics_freshness(record, 1_007_000, motion="walk_leo")
        self.assertTrue(freshness["fresh"])
        self.assertAlmostEqual(freshness["receive_age_ms"], 0.007)
        self.assertAlmostEqual(freshness["age_ms"], 0.257)
        self.assertEqual(freshness["source_age_ms"], 0.25)

    def test_old_latest_message_is_not_current_command(self):
        record = record_safety.leo_command_diagnostics_record(self.message(), 1_000_000, 0)
        freshness = record_safety.leo_command_diagnostics_freshness(record, 151_000_000, motion="walk_leo")
        self.assertFalse(freshness["fresh"])

    def test_impact_gait_keeps_posture_and_its_own_command_provenance(self):
        recorder = record_safety.SafetyRecorder.__new__(record_safety.SafetyRecorder)
        evidence = {}
        recorder.add_posture_evidence(evidence, {"motion": "walk_impact"})
        self.assertEqual(evidence["posture_context"]["reason"], "imu_info_unavailable")
        message = self.message()
        message.source_param_tag = "rl_walking_leolab_impact_v2"
        record = record_safety.leo_command_diagnostics_record(message, 1_000_000, 0)
        self.assertTrue(record_safety.leo_command_diagnostics_freshness(
            record, 1_000_000, motion="walk_impact")["fresh"])
        self.assertFalse(record_safety.leo_command_diagnostics_freshness(
            record, 1_000_000, motion="walk_leo")["fresh"])

    def test_missing_diagnostics_is_non_fresh_and_non_fatal(self):
        self.assertEqual(
            record_safety.leo_command_diagnostics_freshness(None, 100),
            {"available": False, "fresh": False, "age_ms": None},
        )

    def test_stale_source_is_not_fresh_just_because_received_now(self):
        record = record_safety.leo_command_diagnostics_record(self.message(), 1_000_000, 0)
        record["source_age_ms"] = 150.0
        freshness = record_safety.leo_command_diagnostics_freshness(record, 1_000_000, motion="walk_leo")
        self.assertFalse(freshness["fresh"])
        self.assertEqual(freshness["age_ms"], 150.0)

    def test_inactive_or_wrong_motion_is_not_current_command(self):
        record = record_safety.leo_command_diagnostics_record(self.message(), 1_000_000, 0)
        for motion in (None, "passive", "walk", "walk_leo_terrain"):
            with self.subTest(motion=motion):
                freshness = record_safety.leo_command_diagnostics_freshness(record, 1_000_000, motion=motion)
                self.assertFalse(freshness["fresh"])
                self.assertFalse(freshness["motion_matches"])
        record["active"] = False
        self.assertFalse(record_safety.leo_command_diagnostics_freshness(
            record, 1_000_000, motion="walk_leo")["fresh"])
        record["active"] = True
        record["source_param_tag"] = "rl_walking_leolab_terrain_example"
        self.assertTrue(record_safety.leo_command_diagnostics_freshness(
            record, 1_000_000, motion="walk_leo_terrain")["fresh"])

    def test_missing_negative_or_nonfinite_age_is_not_fresh(self):
        for key, value in (("source_age_ms", None), ("source_age_ms", -1),
                           ("source_age_ms", float("nan")), ("source_age_ms", float("inf")),
                           ("receive_monotonic_ns", None), ("receive_monotonic_ns", -1),
                           ("receive_monotonic_ns", 2_000_000)):
            with self.subTest(key=key, value=value):
                record = record_safety.leo_command_diagnostics_record(self.message(), 1_000_000, 0)
                record[key] = value
                self.assertFalse(record_safety.leo_command_diagnostics_freshness(
                    record, 1_000_000, motion="walk_leo")["fresh"])

    def test_duplicate_callback_does_not_refresh_receipt_time(self):
        recorder = record_safety.SafetyRecorder.__new__(record_safety.SafetyRecorder)
        recorder.lock = threading.Lock()
        recorder.closed = False
        recorder.latest = {}
        recorder.receipt_times = lambda: (1_000_000, 0)
        recorder.on_leo_command_diagnostics(self.message(100))
        recorder.receipt_times = lambda: (500_000_000, 0)
        recorder.on_leo_command_diagnostics(self.message(100))
        self.assertEqual(recorder.latest["leo_command_diagnostics"]["receive_monotonic_ns"], 1_000_000)
        recorder.on_leo_command_diagnostics(self.message(101))
        self.assertEqual(recorder.latest["leo_command_diagnostics"]["receive_monotonic_ns"], 500_000_000)

    def test_event_start_and_peak_preserve_speed_chain_without_claiming_actor_action(self):
        recorder = record_safety.SafetyRecorder.__new__(record_safety.SafetyRecorder)
        recorder.args = SimpleNamespace(leo_command_max_age_ms=100.0)
        recorder.active = {}
        recorder.next_event_id = 1
        recorder.event_record_count = 0
        recorder.events_file = None
        recorder.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=0)))
        written = []
        recorder.write_line = lambda stream, event, **kwargs: written.append(event)
        sample = {"sample_index": 1, "receive_monotonic_ns": 1_000_000,
                  "receive_ros_time_ns": 0, "motion": "walk_leo", "leo_command_diagnostics":
                  record_safety.leo_command_diagnostics_record(self.message(), 1_000_000, 0)}
        details = {"event_type": "model_position_limit_violation", "excess": 0.1}
        recorder.add_leo_command_evidence(details, sample)
        recorder.start_condition("joint:0", details, sample)
        recorder.end_condition(recorder.active.pop("joint:0"), sample, "condition_cleared")
        for evidence in (written[0]["leo_command_diagnostics"], written[1]["peak"]["leo_command_diagnostics"]):
            self.assertEqual(evidence["policy_command"], [10.0, 11.0, 12.0])
            self.assertTrue(evidence["freshness"]["fresh"])
            self.assertEqual(evidence["raw_actor_action"], "not_recorded")

    def test_event_evidence_includes_policy_action_when_available(self):
        recorder = record_safety.SafetyRecorder.__new__(record_safety.SafetyRecorder)
        recorder.args = SimpleNamespace(leo_command_max_age_ms=100.0)
        details = {"event_type": "model_position_limit_violation", "excess": 0.1}
        sample = {
            "receive_monotonic_ns": 1_000_000,
            "motion": "walk_leo",
            "leo_command_diagnostics": record_safety.leo_command_diagnostics_record(
                self.message(), 1_000_000, 0),
            "policy_diagnostics": {
                "receive_monotonic_ns": 1_000_000,
                "source_monotonic_ns": 999_000,
                "source_param_tag": "rl_walking_leolab_example",
                "active": True,
                "policy_output_valid": True,
                "raw_action": [0.2],
                "clipped_action": [0.1],
                "controlled_action": [0.1],
                "q_des_policy": [0.3],
                "q_des_command": [0.4],
            },
        }
        recorder.add_leo_command_evidence(details, sample)
        self.assertEqual(details["raw_actor_action"], [0.2])
        self.assertEqual(details["action_provenance"], "policy_diagnostics.raw_action")

    def test_event_evidence_rejects_stale_or_inactive_policy_action(self):
        recorder = record_safety.SafetyRecorder.__new__(record_safety.SafetyRecorder)
        recorder.args = SimpleNamespace(leo_command_max_age_ms=100.0)
        base_policy = {
            "receive_monotonic_ns": 1_000_000,
            "source_monotonic_ns": 999_000,
            "source_param_tag": "rl_walking_leolab_example",
            "active": True,
            "policy_output_valid": True,
            "raw_action": [0.2],
            "clipped_action": [0.1],
            "controlled_action": [0.1],
            "q_des_policy": [0.3],
            "q_des_command": [0.4],
        }
        sample = {"receive_monotonic_ns": 1_000_000, "motion": "walk_leo",
                  "policy_diagnostics": dict(base_policy)}
        details = {"event_type": "model_position_limit_violation", "excess": 0.1}
        recorder.add_leo_command_evidence(details, sample)
        self.assertEqual(details["raw_actor_action"], [0.2])

        stale_sample = {"receive_monotonic_ns": 201_000_000, "motion": "walk_leo",
                        "policy_diagnostics": dict(base_policy)}
        stale_details = {"event_type": "model_position_limit_violation", "excess": 0.1}
        recorder.add_leo_command_evidence(stale_details, stale_sample)
        self.assertEqual(stale_details["raw_actor_action"], "unavailable")
        self.assertIn("stale_or_missing_receive_time", stale_details["action_provenance"])

        inactive_policy = dict(base_policy)
        inactive_policy["active"] = False
        inactive_sample = {"receive_monotonic_ns": 1_000_000, "motion": "walk_leo",
                           "policy_diagnostics": inactive_policy}
        inactive_details = {"event_type": "model_position_limit_violation", "excess": 0.1}
        recorder.add_leo_command_evidence(inactive_details, inactive_sample)
        self.assertEqual(inactive_details["raw_actor_action"], "unavailable")
        self.assertIn("policy_inactive", inactive_details["action_provenance"])

        invalid_policy = dict(base_policy)
        invalid_policy["policy_output_valid"] = False
        invalid_sample = {"receive_monotonic_ns": 1_000_000, "motion": "walk_leo",
                          "policy_diagnostics": invalid_policy}
        invalid_details = {"event_type": "model_position_limit_violation", "excess": 0.1}
        recorder.add_leo_command_evidence(invalid_details, invalid_sample)
        self.assertEqual(invalid_details["raw_actor_action"], "unavailable")
        self.assertIn("policy_output_invalid", invalid_details["action_provenance"])

    def test_ros_schema_covers_published_source_fields(self):
        repo = Path(__file__).resolve().parents[2]
        schema = repo / "src/protocol/interface_protocol/msg/LeoCommandDiagnostics.msg"
        fields = {line.split()[1] for line in schema.read_text().splitlines()
                  if line.strip() and not line.lstrip().startswith("#")}
        self.assertEqual(fields, {"header", "source_monotonic_ns", "source_param_tag", "active",
                                  "raw_stick", "target_tactical", "shaped_tactical", "policy_command",
                                  "source_age_ms"})


class GamepadRecordTest(unittest.TestCase):
    def test_raw_gamepad_topic_is_kept_with_local_receipt_time(self):
        recorder = record_safety.SafetyRecorder.__new__(record_safety.SafetyRecorder)
        recorder.lock = threading.Lock()
        recorder.closed = False
        recorder.latest = {}
        recorder.receipt_times = lambda: (1_000_000, 2_000_000)
        message = SimpleNamespace(
            hardware_connected=True,
            digital_states=[1, 0, 0, 0],
            analog_states=[0.1, -0.2],
            header=SimpleNamespace(stamp=SimpleNamespace(sec=3, nanosec=4)),
        )
        recorder.on_gamepad_keys(message)
        record = recorder.latest["gamepad_keys"]
        self.assertTrue(record["hardware_connected"])
        self.assertEqual(record["digital_states"], [1, 0, 0, 0])
        self.assertEqual(record["analog_states"], [0.1, -0.2])
        self.assertEqual(record["ros_stamp_ns"], 3_000_000_004)

    def test_rc02_diagnostics_preserves_raw_and_normalized_values(self):
        recorder = record_safety.SafetyRecorder.__new__(record_safety.SafetyRecorder)
        recorder.lock = threading.Lock()
        recorder.closed = False
        recorder.latest = {}
        recorder.receipt_times = lambda: (5_000_000, 6_000_000)
        message = SimpleNamespace(
            raw_frame_size=25,
            raw_frame=[0xAA, 0x15, 0x0B] + [0] * 22,
            source_monotonic_ns=4_000_000,
            connected=True,
            frame_valid=True,
            control_id=11,
            heartbeat=7,
            l2_raw=128,
            r2_raw=64,
            stick_raw=[1, -2, 3, -4],
            buttons=[0xFA, 0x7F, 0xFF],
            motion_command=0x12,
            skill_command=0x34,
            reserved=[0] * 8,
            normalized_analog=[0.5, 0.25, 0.1, -0.2, 0.3, -0.4],
            header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=9)),
        )
        recorder.on_rc02_diagnostics(message)
        record = recorder.latest["rc02_diagnostics"]
        self.assertEqual(record["raw_frame_size"], 25)
        self.assertEqual(record["raw_frame"][:3], [0xAA, 0x15, 0x0B])
        self.assertEqual(record["source_monotonic_ns"], 4_000_000)
        self.assertEqual(record["l2_raw"], 128)
        self.assertEqual(record["motion_command"], 0x12)
        self.assertEqual(record["normalized_analog"][1], 0.25)


class SimStateDecodeTest(unittest.TestCase):
    def test_sim_state_decoder_keeps_sim_time_and_contact_force(self):
        values = [struct.pack(">d", 1.25), struct.pack(">i", 1)]
        for array in ([0.1], [0.2], [0.3], [0.0] * 3, [0.0] * 3, [1.0, 0.0, 0.0, 0.0],
                      [0.0] * 3, [0.0] * 3, [0.0] * 3, [1.0, 0.0, 0.0, 0.0],
                      [0.0] * 3, [1.0, 0.0, 0.0, 0.0], [0.0] * 3, [0.0] * 3):
            values.append(struct.pack(f">{len(array)}d", *array))
        values.extend([struct.pack(">i", 6), struct.pack(">6d", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)])
        payload = struct.pack(">Q", record_safety.LCM_SIM_STATE_HASH) + b"".join(values)
        record = record_safety.decode_lcm_sim_state(payload)
        self.assertIsNotNone(record)
        self.assertEqual(record["sim_time_s"], 1.25)
        self.assertEqual(record["joint_position"], [0.1])
        self.assertEqual(record["contact_force"], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])


class PolicyDiagnosticsRecordTest(unittest.TestCase):
    def test_policy_diagnostics_keeps_pre_and_post_transition_fields(self):
        recorder = record_safety.SafetyRecorder.__new__(record_safety.SafetyRecorder)
        recorder.lock = threading.Lock()
        recorder.closed = False
        recorder.latest = {}
        recorder.receipt_times = lambda: (10_000, 20_000)
        message = SimpleNamespace(
            source_monotonic_ns=9_000,
            source_param_tag="rl_walking_leolab_example",
            active=True,
            policy_output_valid=True,
            policy_recurrent=False,
            entry_transition_active=True,
            entry_transition_elapsed=0.04,
            entry_transition_duration=0.16,
            transformed_base_angular_velocity=[1.0, 2.0, 3.0],
            projected_gravity=[0.0, 0.0, -1.0],
            observation_single=[1.0],
            observation=[1.0, 2.0],
            raw_action=[0.2],
            clipped_action=[0.1],
            controlled_action=[0.1],
            q_des_policy=[0.3],
            q_des_command=[0.4],
            qd_des_policy=[0.0],
            qd_des_command=[0.5],
            feed_forward_torque=[0.0],
            stiffness=[10.0],
            damping=[1.0],
            policy_command=[1.0, 2.0, 3.0],
            header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=30)),
        )
        recorder.on_policy_diagnostics(message)
        record = recorder.latest["policy_diagnostics"]
        self.assertEqual(record["q_des_policy"], [0.3])
        self.assertEqual(record["q_des_command"], [0.4])
        self.assertTrue(record["entry_transition_active"])

    def test_control_diagnostics_decoder_splits_nominal_effective_and_noise(self):
        payload = struct.pack(">Qd i 8d", record_safety.LCM_DYNAMIC_VECTOR_DOUBLE_HASH,
                              2.5, 8, 1.0, 2.0, 1.1, 2.2, 0.1, 0.2, 3.5, 1.0)
        record = record_safety.decode_lcm_dynamic_vector(payload)
        self.assertEqual(record["sim_time_s"], 2.5)
        self.assertEqual(record["nominal_ctrl"], [1.0, 2.0])
        self.assertEqual(record["effective_ctrl"], [1.1, 2.2])
        self.assertEqual(record["injected_noise"], [0.1, 0.2])
        self.assertEqual(record["command_age_ms"], 3.5)
        self.assertTrue(record["command_valid"])


if __name__ == "__main__":
    unittest.main()

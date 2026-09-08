import sys
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

    def test_ros_schema_covers_published_source_fields(self):
        repo = Path(__file__).resolve().parents[2]
        schema = repo / "src/protocol/interface_protocol/msg/LeoCommandDiagnostics.msg"
        fields = {line.split()[1] for line in schema.read_text().splitlines()
                  if line.strip() and not line.lstrip().startswith("#")}
        self.assertEqual(fields, {"header", "source_monotonic_ns", "source_param_tag", "active",
                                  "raw_stick", "target_tactical", "shaped_tactical", "policy_command",
                                  "source_age_ms"})


if __name__ == "__main__":
    unittest.main()

"""Offscreen regressions; run with python3 -B -m unittest discover -s tools/virtual_gamepad."""

from contextlib import redirect_stderr
import io
import json
import os
from pathlib import Path
import struct
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parent))

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont, QFontDatabase, QFontMetrics
from PyQt6.QtTest import QTest
from PyQt6.QtWidgets import QApplication

from core.lcm_manager import LcmManager
from lcm_msgs.data import GamepadKeys
from ui.numbered_actions_widget import NumberedActionsWidget
from ui.virtual_gamepad_widget import VirtualGamepadWidget
from virtual_gamepad import VirtualGamepadWindow, configure_chinese_font, parse_args


def state_payload(name):
    value = name.encode("utf-8") + b"\0"
    return struct.pack(">QI", 0x104561AA026A2B0E, len(value)) + value


class FakeLcm:
    def __init__(self):
        self.published = []
        self.subscriptions = {}
        self.next_subscription = 0
        self.incoming = []

    def publish(self, channel, payload):
        self.published.append((channel, payload))

    def subscribe(self, channel, callback):
        self.next_subscription += 1
        self.subscriptions[self.next_subscription] = (channel, callback)
        return self.next_subscription

    def unsubscribe(self, subscription):
        del self.subscriptions[subscription]

    def handle_timeout(self, timeout):
        if not self.incoming:
            return 0
        channel, payload = self.incoming.pop(0)
        for selected, callback in list(self.subscriptions.values()):
            if selected == channel:
                callback(channel, payload)
        return 1


class ChineseFontTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.original_font = QFont(self.app.font())
        self.addCleanup(self.app.setFont, self.original_font)

    def test_missing_font_explains_repair_in_readable_english(self):
        output = io.StringIO()
        with patch("virtual_gamepad.QFontDatabase") as database, redirect_stderr(output):
            database.families.return_value = []
            self.assertFalse(configure_chinese_font(self.app))
        self.assertIn("Chinese font missing", output.getvalue())
        self.assertIn("fonts-noto-cjk", output.getvalue())
        self.assertTrue(output.getvalue().isascii())
        self.assertEqual(self.app.font(), self.original_font)

    def test_preferred_chinese_font_preserves_existing_size_and_fallback(self):
        with patch("virtual_gamepad.QFontDatabase") as database:
            database.families.return_value = ["Droid Sans Fallback", "Noto Sans CJK SC"]
            self.assertTrue(configure_chinese_font(self.app))
        self.assertEqual(self.app.font().families()[0], "Noto Sans CJK SC")
        self.assertEqual(self.app.font().pointSizeF(), self.original_font.pointSizeF())
        self.assertTrue(set(self.original_font.families()).issubset(self.app.font().families()))

    def test_available_font_renders_test_page_and_current_action_names(self):
        families = QFontDatabase.families(QFontDatabase.WritingSystem.SimplifiedChinese)
        with redirect_stderr(io.StringIO()):
            self.assertEqual(configure_chinese_font(self.app), bool(families))
        if families:
            metrics = QFontMetrics(self.app.font())
            for character in "策略测试左勾拳拉锤左直拳左抖拳中启用发送当前机器人状态":
                with self.subTest(character=character):
                    self.assertTrue(metrics.inFontUcs4(ord(character)))


class NumberedActionsUiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        actions = []
        for number in range(1, 28):
            package = f"rl_supermimic_test_{number:02d}"
            actions.append({"id": number, "label": f"动作{number}", "batch": 1,
                            "iteration": number * 2000, "package": package})
            for relative in ("default.yaml", "policies/policy.mnn",
                             "trajectories/supermimic_reference.npz"):
                path = root / package / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b"test fixture")
        self.catalog_path = root / "catalog.json"
        self.catalog_path.write_text(json.dumps({"schema_version": 1, "capacity": 50,
                                                "actions": actions}))
        self.manager = LcmManager()
        self.manager._subscribers = []
        self.lcm = FakeLcm()
        self.manager._lcm_handle = self.lcm
        self.gamepad = VirtualGamepadWidget()
        self.page = NumberedActionsWidget(self.gamepad, self.catalog_path)
        self.page.timer.stop()
        self.addCleanup(self.close_widgets)

    def close_widgets(self):
        self.page.close()
        self.gamepad.close()
        self.manager._lcm_handle = None
        self.manager._subscribers = []

    def enable(self):
        self.gamepad.switch.setChecked(True)
        self.gamepad.timer.stop()

    def ready(self):
        self.enable()
        self.state("walk_leo")

    def state(self, name):
        self.lcm.incoming.append(("task_state", state_payload(name)))
        self.page.poll_state()

    def sent_state(self):
        channel, payload = self.lcm.published[-1]
        self.assertEqual(channel, "virtual_gamepad/gamepad_keys")
        return GamepadKeys.decode(payload)

    def test_fifty_buttons_only_configured_entries_can_execute(self):
        self.assertEqual(len(self.page.action_buttons), 50)
        self.assertFalse(any(button.isEnabled() for button in self.page.action_buttons.values()))
        self.ready()
        self.assertEqual([n for n, button in self.page.action_buttons.items() if button.isEnabled()],
                         list(range(1, 28)))
        self.assertIn("2000轮", self.page.action_buttons[1].text())
        self.assertFalse(self.page.execute_action(28))

    def test_missing_local_catalog_keeps_ordinary_gamepad_controls(self):
        self.page.close()
        self.page = NumberedActionsWidget(self.gamepad)
        self.page.timer.stop()
        self.ready()
        self.assertFalse(self.page.can_execute())
        self.assertFalse(self.page.execute_button.isEnabled())
        self.assertFalse(any(button.isEnabled() for button in self.page.action_buttons.values()))
        self.assertIn("本地测试动作", self.page._catalog_error)
        self.assertTrue(self.page.recovery_buttons["走路 LB+A"].isEnabled())
        self.page.recovery_buttons["走路 LB+A"].click()
        self.assertEqual(tuple(self.gamepad.marcro_button_combination), ("LB", "A"))

    def test_impact_gait_uses_lb_y_and_allows_numbered_actions(self):
        self.assertEqual(self.gamepad.macros["walk_impact: [LB, Y]"], ("LB", "Y"))
        self.assertNotIn("walk_custom: [LB, Y]", self.gamepad.macros)
        self.enable()
        self.state("walk_impact")
        self.assertTrue(self.page.can_execute())

    def test_only_configured_numbered_buttons_are_shown(self):
        self.page.show()
        self.app.processEvents()
        self.assertEqual([number for number, button in self.page.action_buttons.items()
                          if button.isVisible()], list(range(1, 28)))
        self.assertFalse(self.page.execute_action(28))

    def test_enable_connection_fresh_state_and_ready_motion_are_required(self):
        self.state("walk_leo")
        self.assertFalse(self.page.execute_action(1))
        self.enable()
        self.assertFalse(self.page.execute_action(1))
        self.state("passive")
        self.assertFalse(self.page.execute_action(1))
        self.state("walk_leo")
        self.page.last_state_at = time.monotonic() - 2
        self.assertFalse(self.page.execute_action(1))
        self.state("test_action_03")
        self.assertFalse(self.page.execute_action(1))
        self.state("walk_leo")
        self.manager._lcm_handle = None
        self.manager._notify_connection_status(False)
        self.assertFalse(self.page.execute_action(1))

    def test_number_command_centers_sticks_and_starts_with_neutral_release(self):
        self.ready()
        self.gamepad.left_stick_x.setValue(10)
        self.gamepad.gamepad_keys.digital_states[7] = 1
        self.assertTrue(self.page.execute_action(1))
        self.gamepad.send_state()
        neutral = self.sent_state()
        self.assertEqual(list(neutral.digital_states), [0] * 12)
        self.assertEqual(list(neutral.analog_states), [0.0] * 6)
        self.gamepad.send_state()
        command = self.sent_state()
        self.assertEqual(list(command.digital_states), [1] + [0] * 11)
        self.assertEqual(list(command.analog_states), [1.0, 1.0, 0.0, 0.0, 0.0, 0.0])
        self.assertFalse(self.page.execute_action(2))

    def test_only_actual_confirmation_advances_selection_without_executing_next(self):
        self.ready()
        self.page.execute_action(1)
        self.assertEqual(self.page.selected_id, 1)
        self.assertNotIn("执行中", self.page.status_label.text())
        self.state("walk_leo")
        self.assertEqual(self.page.pending_id, 1)
        self.state("test_action_01")
        self.assertIsNone(self.page.pending_id)
        self.assertEqual(self.page.selected_id, 2)
        self.assertIn("执行中 01", self.page.status_label.text())
        self.assertEqual(self.gamepad.should_send_macro_count, 0)
        self.assertEqual(self.lcm.published, [])
        self.assertFalse(self.page.can_execute())
        self.state("walk_leo")
        self.assertTrue(self.page.can_execute())
        self.assertEqual(self.gamepad.should_send_macro_count, 0)

    def test_same_number_can_be_replayed_after_ready_feedback(self):
        self.ready()
        self.page.execute_action(1)
        self.state("test_action_01")
        self.state("walk_leo")
        self.assertTrue(self.page.execute_action(1))
        self.assertEqual(self.gamepad.neutral_macro_frames, 1)

    def test_unconfirmed_action_times_out_and_does_not_retry(self):
        self.ready()
        self.page.execute_action(1)
        self.page.pending_since = time.monotonic() - 4
        self.page.poll_state()
        self.assertIn("未收到进入动作确认", self.page.status_label.text())
        self.assertIsNone(self.page.pending_id)
        self.assertEqual(self.gamepad.should_send_macro_count, 0)
        self.page.poll_state()
        self.assertEqual(self.lcm.published, [])

    def test_expired_feedback_cancels_pending_action(self):
        self.ready()
        self.page.execute_action(1)
        self.page.last_state_at = time.monotonic() - 2
        self.page.poll_state()
        self.assertIn("反馈已过期", self.page.status_label.text())
        self.assertIsNone(self.page.pending_id)
        self.assertEqual(self.gamepad.should_send_macro_count, 0)
        self.assertFalse(self.page.execute_button.isEnabled())

    def test_disconnect_and_disable_clear_commands_without_replay(self):
        self.ready()
        self.page.execute_action(1)
        self.old_lcm = self.lcm
        self.manager._lcm_handle = None
        self.manager._notify_connection_status(False)
        self.assertFalse(self.old_lcm.subscriptions)
        self.assertIsNone(self.page.pending_id)
        self.assertEqual(self.gamepad.should_send_macro_count, 0)
        self.lcm = FakeLcm()
        self.manager._lcm_handle = self.lcm
        self.manager._notify_connection_status(True)
        self.gamepad.send_state()
        self.assertEqual(list(self.sent_state().digital_states), [0] * 12)
        self.state("walk_leo")
        self.page.execute_action(2)
        self.page.enable_switch.setChecked(False)
        self.assertFalse(self.gamepad.is_sending)
        self.assertIsNone(self.page.pending_id)
        self.assertEqual(self.gamepad.should_send_macro_count, 0)

    def test_recovery_replaces_pending_action_and_passive_is_immediate(self):
        self.ready()
        self.page.execute_action(1)
        self.assertTrue(self.page.send_recovery(("LB", "RB")))
        self.assertIsNone(self.page.pending_id)
        self.assertEqual(self.gamepad.neutral_macro_frames, 0)
        self.gamepad.send_state()
        self.assertEqual(list(self.sent_state().digital_states), [1, 1] + [0] * 10)
        self.page.poll_state()
        self.assertEqual(self.gamepad.should_send_macro_count, 9)

    def test_bad_feedback_does_not_unlock_actions(self):
        self.enable()
        self.page.handle_task_state("task_state", b"invalid")
        self.assertIsNone(self.page.current_motion)
        self.assertFalse(self.page.can_execute())

    def test_original_tab_macro_replaces_pending_request_without_being_erased(self):
        self.ready()
        self.page.execute_action(1)
        self.gamepad.send_macro(("LT", "A"), reset_inputs=False)
        self.page.pending_since = time.monotonic() - 4
        self.page.poll_state()
        self.assertIsNone(self.page.pending_id)
        self.assertEqual(self.gamepad.marcro_button_combination, ("LT", "A"))
        self.assertEqual(self.gamepad.should_send_macro_count, 10)

    def test_queued_old_ack_cannot_erase_original_tab_recovery_macro(self):
        self.ready()
        for combination, expected_buttons, expected_analog in (
            (("LT", "A"), [0, 0, 1] + [0] * 9, [1.0] + [0.0] * 5),
            (("LB", "RB"), [1, 1] + [0] * 10, [0.0] * 6),
        ):
            with self.subTest(combination=combination):
                self.state("walk_leo")
                self.assertTrue(self.page.execute_action(1))
                self.gamepad.send_macro(combination, reset_inputs=False)
                self.lcm.incoming.append(("task_state", state_payload("test_action_01")))
                self.page.poll_state()
                self.assertIsNone(self.page.pending_id)
                self.assertEqual(self.page.selected_id, 1)
                self.assertEqual(self.gamepad.marcro_button_combination, combination)
                self.assertEqual(self.gamepad.should_send_macro_count, 10)
                self.gamepad.send_state()
                command = self.sent_state()
                self.assertEqual(list(command.digital_states), expected_buttons)
                self.assertEqual(list(command.analog_states), expected_analog)

    def test_ack_preserves_manual_buttons_and_walking_input_pressed_after_request(self):
        self.ready()
        for manual_input in ("passive", "walking"):
            with self.subTest(manual_input=manual_input):
                self.state("walk_leo")
                self.assertTrue(self.page.execute_action(1))
                if manual_input == "passive":
                    self.gamepad.handle_shortcut_pressed("LB")
                    self.gamepad.handle_shortcut_pressed("RB")
                else:
                    self.gamepad.keyboard_motion_states[Qt.Key.Key_Up] = True
                    self.gamepad.update_keyboard_motion_axes()
                self.state("test_action_01")
                self.assertIsNone(self.page.pending_id)
                self.assertEqual(self.gamepad.should_send_macro_count, 0)
                self.assertEqual(self.gamepad.neutral_macro_frames, 0)
                self.gamepad.send_state()
                command = self.sent_state()
                if manual_input == "passive":
                    self.assertTrue(self.gamepad.shortcut_states["LB"])
                    self.assertTrue(self.gamepad.shortcut_states["RB"])
                    self.assertTrue(self.gamepad.lb_button.isDown())
                    self.assertTrue(self.gamepad.rb_button.isDown())
                    self.assertEqual(list(command.digital_states), [1, 1] + [0] * 10)
                else:
                    self.assertTrue(self.gamepad.keyboard_motion_states[Qt.Key.Key_Up])
                    self.assertEqual(self.gamepad.left_stick_x.value(), 10)
                    self.assertEqual(list(command.analog_states), [0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

    def test_no_catalog_has_no_executable_actions(self):
        page = NumberedActionsWidget(self.gamepad)
        self.addCleanup(page.close)
        page.timer.stop()
        self.assertEqual(page.actions, {})
        self.assertFalse(any(button.isEnabled() for button in page.action_buttons.values()))

    def test_robot_window_title_survives_initialization_and_preserves_qt_options(self):
        args, qt_args = parse_args([
            "--window-title", "Virtual Gamepad - Robot B", "--connect",
            "--lcm-url", "udpm://239.255.76.67:28668?ttl=0", "-platform", "offscreen"])
        self.assertEqual(qt_args, ["-platform", "offscreen"])
        self.assertTrue(args.connect)
        self.assertIsNone(args.test_catalog)
        window = VirtualGamepadWindow(lcm_url=args.lcm_url, window_title=args.window_title)
        self.addCleanup(window.close)
        self.assertEqual(window.windowTitle(), "Virtual Gamepad - Robot B")
        self.assertEqual(window.lcm_manager.url_combo.currentText(), args.lcm_url)
        self.assertFalse(any(button.isEnabled() for button in window.numbered_actions.action_buttons.values()))

    def test_f5_works_with_original_gamepad_tab_hidden(self):
        # Close fixture widgets to avoid another F5 shortcut in the same app.
        self.page.close()
        self.gamepad.close()
        window = VirtualGamepadWindow(test_catalog=self.catalog_path)
        self.addCleanup(window.close)
        window.show()
        window.tab_widget.setCurrentWidget(window.numbered_actions)
        self.app.processEvents()
        window.activateWindow()
        window.numbered_actions.setFocus()
        self.app.processEvents()
        self.assertFalse(window.gamepad_simulator.isVisible())
        QTest.keyClick(window, Qt.Key.Key_F5)
        self.app.processEvents()
        self.assertTrue(window.gamepad_simulator.is_sending)
        self.assertTrue(window.numbered_actions.enable_switch.isChecked())
        QTest.keyClick(window, Qt.Key.Key_F5)
        self.app.processEvents()
        self.assertFalse(window.gamepad_simulator.is_sending)


if __name__ == "__main__":
    unittest.main()

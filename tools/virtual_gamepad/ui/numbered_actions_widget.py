"""Manually test numbered policies using the existing gamepad publisher."""

import time

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (
    QCheckBox, QGridLayout, QHBoxLayout, QLabel, QPushButton,
    QScrollArea, QVBoxLayout, QWidget,
)

from action_catalog import (
    CAPACITY, READY_MOTIONS, action_keys, decode_task_state,
    load_catalog, motion_name,
)
from core.lcm_manager import LcmManager


class NumberedActionsWidget(QWidget):
    STATE_MAX_AGE = 1.0
    ACK_TIMEOUT = 3.0

    def __init__(self, gamepad, catalog_path=None):
        super().__init__()
        self.gamepad = gamepad
        self.actions = {}
        self.selected_id = None
        self.current_motion = None
        self.last_state_at = None
        self.pending_id = None
        self.pending_since = None
        self._lcm = None
        self._subscription = None
        self._catalog_error = (
            "尚未加载本地测试动作，普通遥控仍可使用。请在 "
            "assets/config/t800/numbered_actions/ 配置策略和 catalog.json，"
            "再通过单机或双机虚拟手柄入口重启会话。"
        )
        if catalog_path:
            try:
                self.actions = {entry["id"]: entry
                                for entry in load_catalog(catalog_path, check_assets=True)["actions"]}
                self._catalog_error = "" if self.actions else "测试清单中没有可用策略。"
            except (OSError, ValueError, TypeError, KeyError) as error:
                self._catalog_error = f"测试清单不可用：{error}"
        self._build_ui()
        if self.actions:
            self.select_action(min(self.actions))
        self.gamepad.switch.stateChanged.connect(self._on_enable_changed)
        LcmManager().add_connection_listener(self._on_connection_changed)
        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.poll_state)
        self.timer.start()
        self._sync_connection()
        self.refresh_controls()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        self.enable_switch = QCheckBox("启用发送（F5，与遥控页共用）")
        self.enable_switch.setChecked(self.gamepad.switch.isChecked())
        self.enable_switch.toggled.connect(self.gamepad.switch.setChecked)
        layout.addWidget(self.enable_switch)
        self.current_label = QLabel("当前机器人状态：等待反馈")
        self.selection_label = QLabel("未选择策略")
        self.selection_label.setWordWrap(True)
        self.status_label = QLabel(self._catalog_error or "请进入 PD 站立或走路，再点击编号测试。")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.current_label)
        layout.addWidget(self.selection_label)
        layout.addWidget(self.status_label)

        navigation = QHBoxLayout()
        self.previous_button = QPushButton("上一项")
        self.execute_button = QPushButton("执行所选")
        self.next_button = QPushButton("下一项")
        self.previous_button.clicked.connect(lambda: self.move_selection(-1))
        self.execute_button.clicked.connect(lambda: self.execute_action(self.selected_id))
        self.next_button.clicked.connect(lambda: self.move_selection(1))
        for button in (self.previous_button, self.execute_button, self.next_button):
            navigation.addWidget(button)
        layout.addLayout(navigation)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        grid = QGridLayout(content)
        self.action_buttons = {}
        configured_index = 0
        for number in range(1, CAPACITY + 1):
            entry = self.actions.get(number)
            title = (f"{number:02d}  {entry['label']}\n"
                     f"第{entry['batch']}批 · {entry['iteration']}轮") if entry else f"{number:02d}  预留"
            button = QPushButton(title, content)
            button.setMinimumHeight(64)
            button.setCheckable(True)
            if entry:
                button.setToolTip(entry["package"])
            button.clicked.connect(lambda checked=False, n=number: self.execute_action(n))
            self.action_buttons[number] = button
            if entry:
                grid.addWidget(button, configured_index // 3, configured_index % 3)
                configured_index += 1
            else:
                button.hide()
        scroll.setWidget(content)
        layout.addWidget(scroll)
        layout.addWidget(QLabel(f"已配置 {len(self.actions)} 个动作；编号容量 {CAPACITY}，未配置项隐藏。"))

        recovery = QHBoxLayout()
        self.recovery_buttons = {}
        for label, combination in (
            ("走路 LB+A", ("LB", "A")),
            ("PD 站立 LT+A", ("LT", "A")),
            ("被动 passive LB+RB", ("LB", "RB")),
        ):
            button = QPushButton(label)
            button.clicked.connect(lambda checked=False, keys=combination: self.send_recovery(keys))
            self.recovery_buttons[label] = button
            recovery.addWidget(button)
        layout.addLayout(recovery)
        layout.addWidget(QLabel("编号点击后只执行一次；确认进入动作后选中下一项，不会自动执行下一项。"))

    def select_action(self, number):
        if number not in self.actions:
            return
        self.selected_id = number
        entry = self.actions[number]
        self.selection_label.setText(
            f"所选 {number:02d}：{entry['label']} · 第{entry['batch']}批 · {entry['iteration']}轮"
        )
        for candidate, button in self.action_buttons.items():
            button.setChecked(candidate == number)
        self.refresh_controls()

    def move_selection(self, direction):
        numbers = sorted(self.actions)
        if not numbers:
            return
        index = numbers.index(self.selected_id) if self.selected_id in numbers else 0
        self.select_action(numbers[max(0, min(len(numbers) - 1, index + direction))])

    def _fresh_state(self):
        return (self.last_state_at is not None
                and time.monotonic() - self.last_state_at <= self.STATE_MAX_AGE)

    def can_execute(self):
        return (bool(self.actions) and self.gamepad.is_sending
                and LcmManager().is_connected() and self._fresh_state()
                and self.current_motion in READY_MOTIONS and self.pending_id is None)

    def refresh_controls(self):
        ready = self.can_execute()
        for number, button in self.action_buttons.items():
            button.setEnabled(ready and number in self.actions)
        self.execute_button.setEnabled(ready and self.selected_id in self.actions)
        numbers = sorted(self.actions)
        self.previous_button.setEnabled(bool(numbers) and self.selected_id != numbers[0])
        self.next_button.setEnabled(bool(numbers) and self.selected_id != numbers[-1])
        connected = self.gamepad.is_sending and LcmManager().is_connected()
        for button in self.recovery_buttons.values():
            button.setEnabled(connected)

    def execute_action(self, number):
        if number not in self.actions or not self.can_execute():
            return False
        self.select_action(number)
        if not self.gamepad.send_macro(action_keys(number)):
            self.status_label.setText("发送失败：请检查连接与 Enable。")
            return False
        self.pending_id = number
        self.pending_since = time.monotonic()
        self.status_label.setText(f"已请求 {number:02d}，等待机器人确认进入动作……")
        self.refresh_controls()
        return True

    def send_recovery(self, combination):
        if not self.gamepad.send_macro(combination):
            return False
        self.pending_id = None
        self.pending_since = None
        self.status_label.setText("已发送恢复指令，等待机器人状态反馈。")
        self.refresh_controls()
        return True

    def _on_enable_changed(self, state):
        self.enable_switch.blockSignals(True)
        self.enable_switch.setChecked(bool(state))
        self.enable_switch.blockSignals(False)
        self._clear_pending()
        self.current_motion = None
        self.last_state_at = None
        self.current_label.setText("当前机器人状态：等待新的反馈")
        self.status_label.setText(self._catalog_error or (
            "发送已启用，等待机器人状态反馈。" if state else "发送已关闭，待发动作已取消。"
        ))
        self.refresh_controls()

    def _clear_pending(self):
        self.pending_id = None
        self.pending_since = None
        self.gamepad.discard_pending_inputs()

    def _on_connection_changed(self, connected):
        self._sync_connection()

    def _sync_connection(self):
        handle = LcmManager().lcm_handle
        if handle is self._lcm:
            return
        self._detach_subscription()
        self._clear_pending()
        self.current_motion = None
        self.last_state_at = None
        self.current_label.setText("当前机器人状态：等待反馈" if handle else "当前机器人状态：连接已断开")
        self.status_label.setText(self._catalog_error or "连接变化，待发动作已取消；等待新的状态反馈。")
        self._lcm = handle
        if handle is not None:
            try:
                self._subscription = handle.subscribe("task_state", self.handle_task_state)
            except Exception as error:
                self.status_label.setText(f"无法订阅机器人状态：{error}")
                self._lcm = None
        self.refresh_controls()

    def _detach_subscription(self):
        if self._lcm is not None and self._subscription is not None:
            try:
                self._lcm.unsubscribe(self._subscription)
            except Exception:
                pass
        self._subscription = None
        self._lcm = None

    def handle_task_state(self, channel, payload):
        try:
            name = decode_task_state(payload)
        except (ValueError, UnicodeError):
            return
        previous = self.current_motion
        self.current_motion = name
        self.last_state_at = time.monotonic()
        current_number = next((number for number in self.actions
                               if motion_name(number) == name), None)
        title = (f"{current_number:02d} · {self.actions[current_number]['label']} ({name})"
                 if current_number is not None else name)
        self.current_label.setText(f"当前机器人状态：{title}")
        if (self.pending_id is not None
                and tuple(self.gamepad.marcro_button_combination) != action_keys(self.pending_id)):
            # An old ACK can already be queued when the original tab replaces
            # this request. Never erase that newer command, including passive.
            self.pending_id = None
            self.pending_since = None
            self.status_label.setText("编号请求已被其他手动指令替代。")
        elif self.pending_id is not None and name == motion_name(self.pending_id):
            number = self.pending_id
            self.pending_id = None
            self.pending_since = None
            # The ACK releases only this numbered macro. Manual buttons or
            # stick input pressed since the request must remain available.
            self.gamepad.should_send_macro_count = 0
            self.gamepad.neutral_macro_frames = 0
            self.status_label.setText(f"机器人已确认：执行中 {number:02d}。下一项需要手动点击。")
            following = [candidate for candidate in self.actions if candidate > number]
            if following:
                self.select_action(min(following))
        elif (current_number is not None and name != previous and self.pending_id is None
              and not self._catalog_error):
            self.status_label.setText(f"机器人反馈：执行中 {current_number:02d}。")
        elif (name in READY_MOTIONS and previous != name and self.pending_id is None
              and not self._catalog_error):
            self.status_label.setText(f"机器人当前为 {name}，可点击编号开始下一次测试。")
        self.refresh_controls()

    def poll_state(self):
        self._sync_connection()
        if self._lcm is not None:
            try:
                # Bound the drain so incoming traffic never starves the Qt UI.
                for _ in range(64):
                    if self._lcm.handle_timeout(0) <= 0:
                        break
            except Exception as error:
                self._detach_subscription()
                self._clear_pending()
                self.last_state_at = None
                self.status_label.setText(f"状态接收中断，待发动作已取消：{error}")
        if self.pending_id is not None:
            if tuple(self.gamepad.marcro_button_combination) != action_keys(self.pending_id):
                # A macro on the original tab has priority too. Do not let
                # this old request's timeout erase the replacement command.
                self.pending_id = None
                self.pending_since = None
                self.status_label.setText("编号请求已被其他手动指令替代。")
            elif not self._fresh_state():
                self._clear_pending()
                self.status_label.setText("机器人状态反馈已过期，待发动作已取消；不会自动重试。")
            elif time.monotonic() - self.pending_since >= self.ACK_TIMEOUT:
                number = self.pending_id
                self._clear_pending()
                self.status_label.setText(f"{number:02d} 未收到进入动作确认；已取消待发指令，可检查状态后重试。")
        if self.last_state_at is not None and not self._fresh_state():
            self.current_label.setText(f"当前机器人状态：反馈已过期（最后为 {self.current_motion}）")
        self.refresh_controls()

    def closeEvent(self, event):
        self.timer.stop()
        self._detach_subscription()
        self._clear_pending()
        LcmManager().remove_connection_listener(self._on_connection_changed)
        try:
            self.gamepad.switch.stateChanged.disconnect(self._on_enable_changed)
        except (RuntimeError, TypeError):
            pass
        super().closeEvent(event)

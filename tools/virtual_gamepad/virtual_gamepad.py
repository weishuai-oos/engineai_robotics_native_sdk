import os
import signal
import sys
import argparse

from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QIcon
from PyQt6.QtWidgets import QApplication, QMainWindow, QTabWidget, QVBoxLayout, QWidget
from ui.lcm_manager_widget import LcmManagerWidget
from ui.virtual_gamepad_widget import VirtualGamepadWidget
from ui.numbered_actions_widget import NumberedActionsWidget


class VirtualGamepadWindow(QMainWindow):
    def __init__(self, lcm_url=None, auto_connect=False, test_catalog=None,
                 window_title="Virtual Gamepad"):
        super().__init__()
        self.setWindowTitle(window_title)
        self.lcm_url = lcm_url
        self.auto_connect = auto_connect
        self.test_catalog = test_catalog

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        self.init_ui()

    def init_ui(self):
        current_directory = os.path.dirname(os.path.abspath(__file__))
        self.setWindowIcon(
            QIcon(os.path.join(current_directory, 'assets/logo.jpg')))

        self.lcm_manager = LcmManagerWidget()
        if self.lcm_url:
            self.lcm_manager.url_combo.setCurrentText(self.lcm_url)
        self.main_layout.addWidget(self.lcm_manager)

        self.tab_widget = QTabWidget()
        self.tab_widget.setTabPosition(QTabWidget.TabPosition.North)

        self.gamepad_simulator = VirtualGamepadWidget()
        self.tab_widget.addTab(self.gamepad_simulator, "Virtual Gamepad")
        # Keep F5 available while the original gamepad tab is hidden.
        self.gamepad_simulator.lcm_enabled_shortcut.setParent(self)
        self.numbered_actions = NumberedActionsWidget(self.gamepad_simulator, self.test_catalog)
        self.tab_widget.addTab(self.numbered_actions, "策略测试")

        self.main_layout.addWidget(self.tab_widget)

        if self.auto_connect:
            # Let Qt finish constructing/showing the window before displaying
            # a possible connection error dialog from the existing widget.
            QTimer.singleShot(0, self.lcm_manager.toggle_connection)

    def closeEvent(self, event):
        self.numbered_actions.close()
        self.gamepad_simulator.close()
        self.lcm_manager.close()
        super().closeEvent(event)


def signal_handler(signal, frame):
    print("Ctrl+C detected! Exiting...")
    QApplication.quit()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Virtual gamepad publisher")
    parser.add_argument(
        "--lcm-url",
        default="udpm://239.255.76.67:7667?ttl=0",
        help="LCM URL used by the virtual gamepad publisher",
    )
    parser.add_argument(
        "--window-title",
        default="Virtual Gamepad",
        help="window title identifying the controlled robot",
    )
    parser.add_argument(
        "--test-catalog",
        help="numbered policy catalog registered by the simulation launcher",
    )
    parser.add_argument(
        "--connect",
        action="store_true",
        help="connect to --lcm-url automatically after the window starts",
    )
    return parser.parse_known_args(argv)


if __name__ == "__main__":
    # Register the signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    virtual_args, qt_args = parse_args(sys.argv[1:])
    app = QApplication([sys.argv[0], *qt_args])
    window = VirtualGamepadWindow(
        lcm_url=virtual_args.lcm_url,
        auto_connect=virtual_args.connect,
        test_catalog=virtual_args.test_catalog,
        window_title=virtual_args.window_title,
    )
    window.show()
    if sys.platform != "win32":
        signal.siginterrupt(signal.SIGINT, False)

    # Use QTimer to periodically check for signals
    timer = QTimer()
    # Do nothing, just to allow the event loop to handle signals
    timer.timeout.connect(lambda: None)
    timer.start(100)
    sys.exit(app.exec())

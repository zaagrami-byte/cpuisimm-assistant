"""Smoke test Qt hors écran, SANS ROS ni manette : l'interface doit démarrer, rien ne bouge."""
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PySide6.QtWidgets")

from operator_ui.gamepad import GamepadThread  # noqa: E402
from operator_ui.main import MainWindow  # noqa: E402
from operator_ui.model import PadState, Scan, Snapshot  # noqa: E402
from operator_ui.ros_bridge import RosBridge  # noqa: E402
from operator_ui.teleop import load_gamepad_mapping, load_teleop_config  # noqa: E402
from operator_ui.widgets.lidar_tab import scan_points  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture()
def win(app):
    bridge = RosBridge()
    w = MainWindow(bridge, GamepadThread(), load_teleop_config(), load_gamepad_mapping())
    yield w
    w.teleop_tab.timer.stop()


def test_starts_without_ros_and_manual_is_disabled(win):
    win.on_snapshot(win.bridge.snapshot())
    assert not win.teleop_tab.button.isEnabled()
    assert "ROS 2" in win.teleop_tab.status.text() or "manette" in win.teleop_tab.status.text()
    assert "indisponible" in win.state_tab.conn.text() or "non connecté" in win.state_tab.conn.text()


def test_no_publication_without_session(win):
    win.bridge.publish_teleop(0.2, 0.0)       # node None -> ne fait rien, ne plante pas
    assert not win.bridge.session_on


def test_lidar_shows_explicit_absence_of_data(win):
    win.on_snapshot(Snapshot())
    assert "Aucune donnée" in win.lidar_tab.info.text()
    win.lidar_tab.view.grab()                  # rendu sans exception


def test_lidar_renders_real_scan_and_zones(win):
    scan = Scan(np.full(360, 2.0, dtype=np.float32), -np.pi, 2 * np.pi / 360, 0.15, 8.0, "laser_link")
    zone = np.array([[0.385, 0.21], [0.385, -0.21], [0.2, -0.21], [0.2, 0.21]], dtype=np.float32)
    win.on_snapshot(Snapshot(ros_available=True, robot_connected=True, scan=scan, scan_age=0.1,
                             zone_stop=zone))
    assert "360 points" in win.lidar_tab.info.text() and "arrêt" in win.lidar_tab.info.text()
    win.lidar_tab.view.grab()
    pts = scan_points(scan)
    assert np.allclose(np.hypot(pts[:, 0], pts[:, 1]), 2.0)


def test_lidar_filters_invalid_ranges():
    scan = Scan(np.array([np.inf, np.nan, 0.05, 1.0, 20.0], dtype=np.float32), 0.0, 0.1, 0.15, 8.0)
    assert len(scan_points(scan)) == 1


def test_pad_display_and_state_tab_estop(win):
    win.teleop_tab.on_pad(PadState(connected=True, name="Pad", axes=(0.0,) * 6, buttons=(False,) * 8,
                                   stamp=1e12))
    s = Snapshot(ros_available=True, robot_connected=True, safety_state="EMERGENCY_STOP", mux_mode="ESTOP")
    win.on_snapshot(s)
    assert win.rearm_btn.isEnabled()
    assert "ACTIF" in win.state_tab.estop.text() and "ARRÊT" in win.state_tab.mode.text()


def test_emergency_stop_without_ros_is_reported_not_silent(win, qtbot=None):
    got = []
    win.bridge.op_done.connect(lambda *a: got.append(a))
    win.emergency_stop()
    assert got and got[0][0] == "estop" and got[0][1] is False and "AUCUN" in got[0][2]

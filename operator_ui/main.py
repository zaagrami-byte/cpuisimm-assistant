"""Interface opérateur ISIMM (PC).

  python -m operator_ui                     # application
  python -m operator_ui --check-gamepad     # affiche axes/boutons pour vérifier le mapping
Prérequis : source /opt/ros/jazzy/setup.bash ; mêmes ROS_DOMAIN_ID que le Pi.
"""
from __future__ import annotations

import argparse
import sys

from PySide6.QtCore import Qt
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (QApplication, QHBoxLayout, QLabel, QMainWindow, QPushButton, QTabWidget,
                               QVBoxLayout, QWidget)

from .gamepad import GamepadThread, check_cli
from .model import Snapshot
from .ros_bridge import RosBridge
from .teleop import CONFIG_DIR, load_gamepad_mapping, load_teleop_config
from .widgets.lidar_tab import LidarTab
from .widgets.state_tab import StateTab
from .widgets.teleop_tab import TeleopTab


class MainWindow(QMainWindow):
    def __init__(self, bridge: RosBridge, pad: GamepadThread, teleop_cfg, mapping) -> None:
        super().__init__()
        self.setWindowTitle("Robot ISIMM — Supervision et pilotage")
        self.resize(900, 760)
        self.bridge, self.pad = bridge, pad
        self.snap = Snapshot()
        self._rearming = False
        self.state_tab, self.lidar_tab = StateTab(), LidarTab()
        self.teleop_tab = TeleopTab(bridge, teleop_cfg, mapping)
        tabs = QTabWidget()
        tabs.addTab(self.state_tab, "État du robot")
        tabs.addTab(self.lidar_tab, "LiDAR")
        tabs.addTab(self.teleop_tab, "Pilotage manuel")

        # Barre d'arrêt d'urgence : visible en permanence, quel que soit l'onglet.
        self.estop_btn = QPushButton("ARRÊT D'URGENCE  (Échap)")
        self.estop_btn.setMinimumHeight(56)
        self.estop_btn.setStyleSheet("background:#c62828;color:white;font-size:18px;font-weight:bold;")
        self.estop_btn.clicked.connect(self.emergency_stop)
        self.rearm_btn = QPushButton("Réarmer")
        self.rearm_btn.setMinimumHeight(56)
        self.rearm_btn.setEnabled(False)
        self.rearm_btn.clicked.connect(self.rearm)
        self.msg = QLabel("")
        self.msg.setWordWrap(True)
        bar = QHBoxLayout()
        bar.addWidget(self.estop_btn, 3)
        bar.addWidget(self.rearm_btn, 1)
        central = QWidget()
        lay = QVBoxLayout(central)
        lay.addWidget(tabs, 1)
        lay.addLayout(bar)
        lay.addWidget(self.msg)
        self.setCentralWidget(central)

        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self.emergency_stop,
                  context=Qt.ApplicationShortcut)
        bridge.updated.connect(self.on_snapshot)
        bridge.op_done.connect(self.on_op_done)
        pad.state.connect(self.teleop_tab.on_pad)
        self.teleop_tab.message.connect(self.msg.setText)

    def on_snapshot(self, s: Snapshot) -> None:
        self.snap = s
        self.state_tab.update_snapshot(s)
        self.lidar_tab.update_snapshot(s)
        self.teleop_tab.on_snapshot(s)
        self.rearm_btn.setEnabled(s.estop_active and s.ros_available and not self._rearming)

    def emergency_stop(self) -> None:
        self.teleop_tab.force_stop()          # immédiat, local
        self.bridge.trigger_estop()           # répété jusqu'à confirmation
        self.msg.setText("Arrêt d'urgence demandé…")

    def rearm(self) -> None:
        self._rearming = True
        self.rearm_btn.setEnabled(False)
        self.msg.setText("Réarmement en cours (vérification sécurité + contrôleur moteur)…")
        self.bridge.request_rearm()

    def on_op_done(self, op: str, ok: bool, text: str) -> None:
        if op == "rearm":
            self._rearming = False
        if op in ("estop", "rearm"):
            self.msg.setText(text)
            self.msg.setStyleSheet("color:#2e7d32;" if ok else "color:#c62828;font-weight:bold;")

    def closeEvent(self, ev) -> None:
        self.teleop_tab.timer.stop()
        self.bridge.stop_motion()             # zéro + fin de session avant tout
        self.pad.stop()
        self.bridge.shutdown()
        ev.accept()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Interface opérateur ISIMM")
    ap.add_argument("--check-gamepad", action="store_true", help="affiche axes/boutons de la manette")
    ap.add_argument("--config-dir", default=str(CONFIG_DIR))
    args = ap.parse_args(argv)
    if args.check_gamepad:
        return check_cli()
    teleop_cfg = load_teleop_config(f"{args.config_dir}/teleop.yaml")
    mapping = load_gamepad_mapping(f"{args.config_dir}/gamepad.yaml")
    app = QApplication(sys.argv[:1])
    bridge = RosBridge(teleop_cfg.snapshot_stale_s)
    pad = GamepadThread()
    win = MainWindow(bridge, pad, teleop_cfg, mapping)
    bridge.start()                            # si ROS 2 est absent : interface disponible, pilotage désactivé
    pad.start()
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())

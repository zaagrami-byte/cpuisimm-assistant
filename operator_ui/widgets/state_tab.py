"""Onglet 1 — État du robot."""
from __future__ import annotations

from PySide6.QtWidgets import QGridLayout, QGroupBox, QLabel, QVBoxLayout, QWidget

from ..model import Snapshot

NAV2_NODES = ("controller_server", "planner_server", "behavior_server", "velocity_smoother",
              "collision_monitor", "bt_navigator", "waypoint_follower")
LEVEL_STYLE = {0: ("#2e7d32", "OK"), 1: ("#ef6c00", "Avertissement"), 2: ("#c62828", "Défaut"),
               3: ("#757575", "Périmé / absent")}
MODE_TEXT = {"AUTO": ("AUTOMATIQUE", "#1565c0"),
             "AUTO_WAIT": ("AUTOMATIQUE — en attente (aucune reprise d'ancien but)", "#1565c0"),
             "MANUAL": ("MANUEL", "#6a1b9a"), "ESTOP": ("ARRÊT D'URGENCE", "#c62828")}


def component_rows(s: Snapshot) -> list[tuple[str, int, str]]:
    """(libellé, niveau, détail) des composants essentiels, déduits de /robot_status."""
    c = s.components

    def one(label, key):
        lvl, msg = c.get(key, (3, "absent"))
        return (label, lvl, msg)

    rows = [one("LiDAR", "lidar_scan"), one("Odométrie (RF2O)", "odom_rf2o"), one("IMU", "imu"),
            one("Sécurité (safety_node)", "safety_node"), one("Contrôleur moteur", "motor_controller")]
    if s.navigation_mode:
        nav = [c.get(n, (3, "absent")) for n in NAV2_NODES]
        worst = max(lv for lv, _ in nav)
        bad = [n for n, (lv, _) in zip(NAV2_NODES, nav) if lv != 0]
        rows.append(("Nav2", worst, "tous actifs" if worst == 0 else "à vérifier : " + ", ".join(bad)))
        rows.append(one("Multiplexeur de commandes", "cmd_vel_mux"))
    else:
        rows.append(("Nav2", 3, "non lancé (mode base/mapping)"))
    return rows


def fmt(v, unit, nd=2) -> str:
    return "—" if v is None else f"{v:.{nd}f} {unit}"


class StateTab(QWidget):
    def __init__(self) -> None:
        super().__init__()
        lay = QVBoxLayout(self)
        self.conn = QLabel()
        self.mode = QLabel()
        self.mode.setStyleSheet("font-size:20px;font-weight:bold;")
        self.conn.setStyleSheet("font-size:14px;font-weight:bold;")
        lay.addWidget(self.conn)
        lay.addWidget(self.mode)

        box = QGroupBox("Composants")
        self.grid = QGridLayout(box)
        self._rows: list[tuple[QLabel, QLabel, QLabel]] = []
        for i in range(7):
            labels = (QLabel(), QLabel(), QLabel())
            for j, w in enumerate(labels):
                self.grid.addWidget(w, i, j)
            self._rows.append(labels)
        lay.addWidget(box)

        sp = QGroupBox("Vitesses et sécurité")
        g = QGridLayout(sp)
        self.cmd, self.meas = QLabel(), QLabel()
        self.safety, self.motor, self.estop = QLabel(), QLabel(), QLabel()
        for i, (name, w) in enumerate((("Vitesse commandée (vers moteurs)", self.cmd),
                                       ("Vitesse mesurée (odométrie)", self.meas),
                                       ("État de sécurité", self.safety),
                                       ("Liaison moteur / Arduino", self.motor),
                                       ("Arrêt d'urgence", self.estop))):
            g.addWidget(QLabel(name), i, 0)
            g.addWidget(w, i, 1)
        lay.addWidget(sp)
        lay.addStretch(1)
        self.update_snapshot(Snapshot())

    def update_snapshot(self, s: Snapshot) -> None:
        if not s.ros_available:
            self.conn.setText(f"ROS 2 indisponible : {s.ros_error or 'rclpy introuvable'}")
            self.conn.setStyleSheet("color:#c62828;font-size:14px;font-weight:bold;")
        elif not s.robot_connected:
            self.conn.setText("ROS 2 actif — robot NON connecté (aucun /robot_status récent)")
            self.conn.setStyleSheet("color:#ef6c00;font-size:14px;font-weight:bold;")
        else:
            self.conn.setText("Robot connecté")
            self.conn.setStyleSheet("color:#2e7d32;font-size:14px;font-weight:bold;")
        text, color = MODE_TEXT.get(s.mux_mode, ("inconnu (mode de commande non reçu)", "#757575"))
        self.mode.setText(f"Mode de commande : {text}")
        self.mode.setStyleSheet(f"font-size:20px;font-weight:bold;color:{color};")

        rows = component_rows(s) if s.robot_connected else []
        for i, labels in enumerate(self._rows):
            if i < len(rows):
                name, lvl, msg = rows[i]
                color, word = LEVEL_STYLE.get(lvl, LEVEL_STYLE[3])
                labels[0].setText(name)
                labels[1].setText(f"● {word}")
                labels[1].setStyleSheet(f"color:{color};font-weight:bold;")
                labels[2].setText(msg)
            else:
                for w in labels:
                    w.setText("")
        if not rows:
            self._rows[0][0].setText("Aucune donnée : robot non connecté")

        self.cmd.setText(f"{fmt(s.cmd_v, 'm/s')} | {fmt(s.cmd_w, 'rad/s')}")
        self.meas.setText(f"{fmt(s.meas_v, 'm/s')} | {fmt(s.meas_w, 'rad/s')}")
        self.safety.setText(s.safety_state or "inconnu")
        self.motor.setText(s.motor_status or "inconnu")
        self.estop.setText("ACTIF" if s.estop_active else ("inactif" if s.robot_connected else "inconnu"))
        self.estop.setStyleSheet("color:#c62828;font-weight:bold;" if s.estop_active else "")

"""Onglet 3 — Pilotage manuel."""
from __future__ import annotations

import time

from PySide6.QtCore import QTimer, Signal
from PySide6.QtWidgets import (QCheckBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QProgressBar,
                               QPushButton, QRadioButton, QVBoxLayout, QWidget)

from ..model import PadState, Snapshot
from ..teleop import (GamepadMapping, TeleopConfig, TeleopController, manual_availability,
                      session_must_end)

OFF, STARTING, ON, STOPPING = "off", "starting", "on", "stopping"


class TeleopTab(QWidget):
    message = Signal(str)               # message d'état pour la barre du bas

    def __init__(self, bridge, cfg: TeleopConfig, mapping: GamepadMapping) -> None:
        super().__init__()
        self.bridge, self.cfg, self.mapping = bridge, cfg, mapping
        self.ctrl = TeleopController(cfg, mapping)
        self.pad = PadState()
        self.snap = Snapshot()
        self.state = OFF
        self._since = 0.0
        self._cmd = None

        lay = QVBoxLayout(self)
        g = QGroupBox("Manette")
        gl = QGridLayout(g)
        self.pad_name, self.pad_state = QLabel("—"), QLabel()
        gl.addWidget(QLabel("Périphérique"), 0, 0)
        gl.addWidget(self.pad_name, 0, 1)
        gl.addWidget(QLabel("État"), 1, 0)
        gl.addWidget(self.pad_state, 1, 1)
        lay.addWidget(g)

        mg = QGroupBox("Vérification du mapping (bougez les sticks, pressez les boutons)")
        self.mgrid = QGridLayout(mg)
        self._axis_bars: list[QProgressBar] = []
        self._btn_labels: list[QLabel] = []
        self.map_text = QLabel()
        self.map_check = QCheckBox("J'ai vérifié le mapping (axes et LB ci-dessus correspondent à gamepad.yaml)")
        self.map_check.setChecked(mapping.verified)
        self.map_check.setEnabled(not mapping.verified)
        lay.addWidget(mg)
        lay.addWidget(self.map_text)
        lay.addWidget(self.map_check)

        sp = QGroupBox("Vitesse")
        sl = QHBoxLayout(sp)
        self.slow = QRadioButton(f"Lente ({cfg.slow:.2f} m/s)")
        self.normal = QRadioButton(f"Normale ({cfg.normal:.2f} m/s)")
        self.slow.setChecked(True)
        sl.addWidget(self.slow)
        sl.addWidget(self.normal)
        lay.addWidget(sp)

        self.deadman = QLabel()
        self.cmd_label = QLabel()
        self.status = QLabel()
        self.status.setWordWrap(True)
        self.button = QPushButton("Activer le pilotage manuel")
        self.button.setMinimumHeight(44)
        self.button.clicked.connect(self._on_button)
        for w in (self.deadman, self.cmd_label, self.status, self.button):
            lay.addWidget(w)
        lay.addStretch(1)

        bridge.op_done.connect(self._on_op_done)
        self.timer = QTimer(self)
        self.timer.setInterval(int(1000 / cfg.rate_hz))
        self.timer.timeout.connect(self._tick)
        self.timer.start()
        self._refresh()

    # ---------------------------------------------------------------- entrées
    def on_pad(self, pad: PadState) -> None:
        self.pad = pad
        while len(self._axis_bars) < len(pad.axes):
            i = len(self._axis_bars)
            bar = QProgressBar()
            bar.setRange(-100, 100)
            bar.setFormat(f"axe {i} : %v")
            self.mgrid.addWidget(bar, i // 2, i % 2)
            self._axis_bars.append(bar)
        for bar, v in zip(self._axis_bars, pad.axes):
            bar.setValue(int(round(v * 100)))
        while len(self._btn_labels) < len(pad.buttons):
            i = len(self._btn_labels)
            lab = QLabel(f"B{i}")
            self.mgrid.addWidget(lab, 10 + i // 8, i % 8)
            self._btn_labels.append(lab)
        for lab, pressed in zip(self._btn_labels, pad.buttons):
            lab.setStyleSheet("background:#2e7d32;color:white;padding:2px;" if pressed else "padding:2px;")

    def on_snapshot(self, s: Snapshot) -> None:
        self.snap = s

    # ---------------------------------------------------------------- logique
    def mapping_ok(self) -> bool:
        return self.ctrl.mapping_complete() and self.map_check.isChecked()

    def blocking_reason(self) -> str:
        if self.pad.error:
            return self.pad.error
        if not self.pad.connected:
            return "aucune manette détectée"
        if not self.ctrl.mapping_complete():
            return "mapping incomplet dans gamepad.yaml (forward, turn, deadman)"
        if not self.map_check.isChecked():
            return "cochez « J'ai vérifié le mapping » après avoir testé les axes et LB"
        ok, why = manual_availability(self.snap, self.cfg)
        return "" if ok else why

    def speed_mode(self) -> str:
        return "slow" if self.slow.isChecked() else "normal"

    def _tick(self) -> None:
        now = time.monotonic()
        if self.state == ON:
            ended = session_must_end(self.snap, self.cfg, now - self._since)
            self._cmd = self.ctrl.compute(self.pad, self.speed_mode(), now)
            stale = self._cmd.reason in ("manette absente", "données de la manette périmées")
            if ended or stale:
                self._end_session(ended or "perte de la manette")
            else:
                self.bridge.publish_teleop(self._cmd.linear, self._cmd.angular)
        self._refresh()

    def _on_button(self) -> None:
        if self.state == OFF:
            why = self.blocking_reason()
            if why:
                self.status.setText(f"Activation impossible : {why}.")
                return
            self.state = STARTING
            self.status.setText("Annulation des buts Nav2 et vérification…")
            self.bridge.request_manual(True)
        elif self.state == ON:
            self.stop_session("Pilotage manuel désactivé.")

    def _on_op_done(self, op: str, ok: bool, msg: str) -> None:
        if op == "manual_on":
            if ok:
                self.state, self._since = ON, time.monotonic()
            else:
                self.state = OFF
        elif op == "manual_off":
            self.state = OFF
        if op in ("manual_on", "manual_off"):
            self.status.setText(msg)
            self.message.emit(msg)

    def _end_session(self, reason: str) -> None:
        self.stop_session(f"Pilotage manuel arrêté : {reason}.")

    def stop_session(self, text: str = "") -> None:
        """Arrêt immédiat local (zéro + fin de session), puis annulation Nav2 en arrière-plan."""
        if self.state == OFF:
            return
        self.bridge.stop_motion()
        self.state = STOPPING
        if text:
            self.status.setText(text)
            self.message.emit(text)
        self.bridge.request_manual(False)

    def force_stop(self) -> None:
        """Appelé par l'arrêt d'urgence : état local remis à OFF sans attendre."""
        self.bridge.stop_motion()
        self.state = OFF
        self.status.setText("Arrêt d'urgence : pilotage manuel désactivé. Réactivation explicite requise.")

    # ---------------------------------------------------------------- affichage
    def _refresh(self) -> None:
        p = self.pad
        self.pad_name.setText(p.name or "—")
        if p.error:
            self.pad_state.setText(p.error)
        else:
            self.pad_state.setText("connectée" if p.connected else "absente")
        m = self.mapping
        self.map_text.setText(
            f"Mapping configuré : avant/arrière = axe {m.axis_forward}, rotation = axe {m.axis_turn}, "
            f"deadman (LB) = bouton {m.deadman}. {len(p.axes)} axes, {len(p.buttons)} boutons détectés.")
        c = self._cmd if self.state == ON else None
        dm = c.deadman if c else (bool(p.buttons[m.deadman]) if p.connected and m.deadman is not None
                                  and m.deadman < len(p.buttons) else False)
        self.deadman.setText(f"Deadman (LB) : {'MAINTENU' if dm else 'relâché'}")
        self.deadman.setStyleSheet("color:#2e7d32;font-weight:bold;" if dm else "font-weight:bold;")
        self.cmd_label.setText(
            f"Commande envoyée : {c.linear:+.2f} m/s | {c.angular:+.2f} rad/s" +
            (f"  ({c.reason})" if c and c.reason else "") if c else "Commande envoyée : aucune")
        if self.state == OFF:
            why = self.blocking_reason()
            self.button.setEnabled(not why)
            self.button.setText("Activer le pilotage manuel")
            if why:
                self.status.setText(f"Pilotage manuel désactivé : {why}.")
            elif not self.status.text().startswith(("Pilotage manuel activé", "Pilotage manuel désactivé.",
                                                    "Pilotage manuel arrêté", "Arrêt d'urgence")):
                self.status.setText("Prêt. Activez le pilotage manuel, puis maintenez LB pour avancer.")
        else:
            self.button.setEnabled(self.state == ON)
            self.button.setText("Désactiver le pilotage manuel")

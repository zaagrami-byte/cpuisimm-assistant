"""Onglet 2 — LiDAR : scan 2D réel, contour du robot, zoom, zones de sécurité si reçues."""
from __future__ import annotations


import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import (QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget)

from ..model import Snapshot

ROBOT_L, ROBOT_W = 0.57, 0.40          # m (URDF) ; LiDAR au centre (laser_x = laser_y = 0)
STALE_S = 1.5


def scan_points(scan) -> np.ndarray:
    """Points (x avant, y gauche) en mètres, uniquement les mesures valides."""
    r = np.asarray(scan.ranges, dtype=np.float64)
    ang = scan.angle_min + scan.angle_increment * np.arange(r.size)
    ok = np.isfinite(r) & (r >= max(scan.range_min, 0.0)) & (r <= scan.range_max)
    return np.column_stack((r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])))


class ScanView(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(420, 420)
        self.scale = 120.0               # pixels / mètre
        self.snap = Snapshot()

    def set_snapshot(self, s: Snapshot) -> None:
        self.snap = s
        self.update()

    def set_scale(self, px_per_m: float) -> None:
        self.scale = max(20.0, min(600.0, px_per_m))
        self.update()

    def wheelEvent(self, e) -> None:
        self.set_scale(self.scale * (1.15 if e.angleDelta().y() > 0 else 1 / 1.15))

    def _px(self, x: float, y: float) -> QPointF:      # avant du robot vers le haut
        return QPointF(self.width() / 2 - y * self.scale, self.height() / 2 - x * self.scale)

    def paintEvent(self, _) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#101418"))
        s = self.snap
        c = self._px(0, 0)
        step = 0.5 if self.scale > 150 else 1.0
        p.setPen(QPen(QColor("#2a323a"), 1))
        for i in range(1, int(max(self.width(), self.height()) / self.scale / step) + 2):
            r = i * step * self.scale
            p.drawEllipse(c, r, r)
            p.drawText(QPointF(c.x() + 3, c.y() - r - 2), f"{i * step:g} m")
        stale = s.scan is None or s.scan_age is None or s.scan_age > STALE_S
        if not stale:
            p.setPen(QPen(QColor("#ff5252"), 3))
            p.drawPoints(QPolygonF([self._px(x, y) for x, y in scan_points(s.scan)]))
        for poly, col, name in ((s.zone_slow, "#ffd600", "ralentissement"), (s.zone_stop, "#ff1744", "arrêt")):
            if poly is not None:
                pen = QPen(QColor(col), 2, Qt.DashLine)
                p.setPen(pen)
                p.setBrush(Qt.NoBrush)
                p.drawPolygon(QPolygonF([self._px(float(x), float(y)) for x, y in poly]))
        # contour du robot + flèche avant
        p.setPen(QPen(QColor("#4fc3f7"), 2))
        p.setBrush(QColor(79, 195, 247, 40))
        tl, br = self._px(ROBOT_L / 2, ROBOT_W / 2), self._px(-ROBOT_L / 2, -ROBOT_W / 2)
        p.drawRect(QRectF(tl, br).normalized())
        nose = self._px(ROBOT_L / 2 + 0.08, 0)
        p.drawLine(c, nose)
        if stale:
            p.setPen(QColor("#ffb74d"))
            font = p.font()
            font.setPointSize(14)
            font.setBold(True)
            p.setFont(font)
            p.drawText(self.rect(), Qt.AlignCenter,
                       "Aucune donnée LiDAR" if s.scan is None else "Données LiDAR périmées")
        p.end()


class LidarTab(QWidget):
    def __init__(self) -> None:
        super().__init__()
        lay = QVBoxLayout(self)
        self.view = ScanView()
        self.info = QLabel()
        bar = QHBoxLayout()
        minus, plus, fit = QPushButton("−"), QPushButton("+"), QPushButton("Réinitialiser")
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(20, 400)
        self.slider.setValue(120)
        self.slider.valueChanged.connect(self.view.set_scale)
        minus.clicked.connect(lambda: self.slider.setValue(int(self.slider.value() / 1.25)))
        plus.clicked.connect(lambda: self.slider.setValue(int(self.slider.value() * 1.25)))
        fit.clicked.connect(lambda: self.slider.setValue(120))
        for w in (QLabel("Zoom"), minus, self.slider, plus, fit):
            bar.addWidget(w)
        lay.addLayout(bar)
        lay.addWidget(self.view, 1)
        lay.addWidget(self.info)

    def update_snapshot(self, s: Snapshot) -> None:
        self.view.set_snapshot(s)
        zones = [n for z, n in ((s.zone_stop, "arrêt"), (s.zone_slow, "ralentissement")) if z is not None]
        if s.scan is None or s.scan_age is None or s.scan_age > STALE_S:
            self.info.setText("Aucune donnée /scan récente. Rien n'est simulé.")
            return
        pts = scan_points(s.scan)
        nearest = f"{float(np.min(np.hypot(pts[:, 0], pts[:, 1]))):.2f} m" if len(pts) else "—"
        self.info.setText(
            f"{len(pts)} points valides — obstacle le plus proche : {nearest} — "
            f"repère {s.scan.frame_id or '?'} (avant = haut). Zones de sécurité : "
            + (", ".join(zones) if zones else "non disponibles (topics /polygon_* non reçus)"))

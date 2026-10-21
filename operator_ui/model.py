"""Structures de données partagées (aucune dépendance Qt/ROS : testables partout)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class PadState:
    """Dernier état lu de la manette."""
    connected: bool = False
    name: str = ""
    axes: tuple = ()
    buttons: tuple = ()
    stamp: float = 0.0          # time.monotonic() de la lecture
    error: str = ""


@dataclass(frozen=True)
class Scan:
    ranges: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    frame_id: str = ""


@dataclass(frozen=True)
class Snapshot:
    """État du robot vu par l'interface (immuable, produit par le thread ROS)."""
    ros_available: bool = False
    ros_error: str = ""
    robot_connected: bool = False                 # /robot_status récent
    components: dict = field(default_factory=dict)  # nom -> (niveau 0..3, message)
    safety_state: Optional[str] = None
    motor_status: Optional[str] = None
    mux_mode: Optional[str] = None                # AUTO | AUTO_WAIT | MANUAL | ESTOP
    cmd_v: Optional[float] = None                 # /safe_cmd_vel (dernière commande vers les moteurs)
    cmd_w: Optional[float] = None
    meas_v: Optional[float] = None                # /odom (RF2O)
    meas_w: Optional[float] = None
    scan: Optional[Scan] = None
    scan_age: Optional[float] = None
    zone_stop: Optional[np.ndarray] = None        # /polygon_stop (base_link), si reçu
    zone_slow: Optional[np.ndarray] = None
    nav_goals_active: Optional[int] = None        # buts Nav2 actifs (None = inconnu)
    session_on: bool = False

    @property
    def navigation_mode(self) -> bool:
        return "controller_server" in self.components

    @property
    def estop_active(self) -> bool:
        return (self.safety_state == "EMERGENCY_STOP" or self.motor_status == "ARDUINO_ESTOP"
                or self.mux_mode == "ESTOP")

"""Contrat entre les outils IA et le robot. Aucun import ROS ici (testable sans ROS)."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Protocol


class NavState(str, Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    SUCCEEDED = "SUCCEEDED"
    CANCELED = "CANCELED"
    FAILED = "FAILED"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class NavSnapshot:
    state: NavState
    destination: str | None = None
    detail: str = ""
    distance_remaining: float | None = None


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float  # radians


class RobotError(Exception): ...
class RobotUnavailableError(RobotError): """Robot/Pi injoignable (DDS, réseau)."""
class PoseUnavailableError(RobotError): """TF map->base_link indisponible."""
class Nav2UnavailableError(RobotError): """Serveur d'action Nav2 indisponible."""
class NavigationError(RobotError): """Échec d'envoi/annulation d'un but."""


class RobotInterface(Protocol):
    def get_status(self) -> dict: ...
    def get_pose(self) -> Pose2D: ...
    def get_observations(self) -> list[str]: ...
    def is_moving(self) -> bool | None: ...
    def nav_snapshot(self) -> NavSnapshot: ...
    def navigate(self, name: str, x: float, y: float, yaw: float) -> NavSnapshot: ...
    def cancel_navigation(self) -> bool: ...
    def emergency_stop(self) -> bool: ...
    def add_nav_listener(self, callback: Callable[[NavSnapshot], None]) -> None: ...
    def close(self) -> None: ...


class OfflineRobot:
    """Utilisé avec --no-ros : tout ce qui touche au robot échoue proprement."""

    def _down(self):
        raise RobotUnavailableError("mode --no-ros")

    def get_status(self): self._down()
    def get_pose(self): self._down()
    def get_observations(self): return []
    def is_moving(self): return None
    def nav_snapshot(self): self._down()
    def navigate(self, name, x, y, yaw): self._down()
    def cancel_navigation(self): self._down()
    def emergency_stop(self): self._down()
    def add_nav_listener(self, callback): pass
    def close(self): pass 

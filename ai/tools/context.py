from __future__ import annotations

from dataclasses import dataclass

from ros2_interface.interface import RobotInterface
from settings import Settings
from tools.locations import LocationBook


@dataclass(frozen=True)
class ToolContext:
    settings: Settings
    robot: RobotInterface
    locations: LocationBook 

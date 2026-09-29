from __future__ import annotations

import dataclasses

from assistant.llm_client import LLMReply, ToolCall
from ros2_interface.interface import (NavSnapshot, NavState, Pose2D, RobotUnavailableError)
from settings import load_settings
from tools.locations import Location, LocationBook


def make_settings(**overrides):
    return dataclasses.replace(load_settings(), **overrides)


def make_locations() -> LocationBook:
    return LocationBook({
        "accueil": Location("accueil", "Hall", 1.0, 2.0, 0.0, ("hall",), "à l'accueil", "l'accueil"),
        "administration": Location("administration", "Admin", 5.0, 1.0, 1.57, ("secrétariat",),
                                   "à l'administration", "l'administration"),
        "laboratoire": Location("laboratoire", "Labo", 8.0, 3.0, 3.14, ("labo",),
                                "au laboratoire", "le laboratoire"),
        "direction": Location("direction", "Dir", None, None, None, (), "à la direction", "la direction"),
    })


class FakeRobot:
    def __init__(self) -> None:
        self.reachable = True
        self.safety_state = "OK"
        self.state = NavState.IDLE
        self.dest: str | None = None
        self.moving: bool | None = False
        self.calls: list[tuple] = []
        self.listeners = []

    def _need(self):
        if not self.reachable:
            raise RobotUnavailableError("down")

    def get_status(self):
        self._need()
        return {"overall_level": 0, "overall_message": "ok", "components": [],
                "safety_state": self.safety_state, "motor_status": "OK"}

    def get_pose(self): self._need(); return Pose2D(1.0, 2.0, 0.0)
    def get_observations(self): return []
    def is_moving(self): return self.moving
    def nav_snapshot(self): return NavSnapshot(self.state, self.dest)

    def navigate(self, name, x, y, yaw):
        self._need()
        self.calls.append(("navigate", name))
        self.state, self.dest = NavState.NAVIGATING, name
        return self.nav_snapshot()

    def cancel_navigation(self):
        self._need()
        self.calls.append(("cancel",))
        was = self.state == NavState.NAVIGATING
        if was:
            self.state = NavState.CANCELED
        return was

    def emergency_stop(self): self._need(); self.calls.append(("estop",)); return True
    def add_nav_listener(self, cb): self.listeners.append(cb)
    def close(self): pass


class FakeLLM:
    """Renvoie des LLMReply scriptés, dans l'ordre."""

    def __init__(self, *replies: LLMReply) -> None:
        self.replies = list(replies)
        self.seen: list[list[dict]] = []

    def chat(self, messages, tools):
        self.seen.append(list(messages))
        return self.replies.pop(0)


def call(name: str, **args) -> LLMReply:
    return LLMReply("", (ToolCall(name, args),)) 

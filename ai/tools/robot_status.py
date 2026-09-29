from __future__ import annotations

from typing import TYPE_CHECKING

from assistant.tool_registry import Tool, ToolRegistry, fail, ok
from ros2_interface.interface import PoseUnavailableError

if TYPE_CHECKING:
    from tools.context import ToolContext

_EMPTY = {"type": "object", "properties": {}, "additionalProperties": False}
_LEVEL_FR = {1: "avertissement", 2: "défaut", 3: "périmé"}


def register(registry: ToolRegistry, ctx: "ToolContext") -> None:
    def get_robot_status() -> dict:
        st = ctx.robot.get_status()   # RobotUnavailableError -> message standard
        problems = [f"{c['name']} ({_LEVEL_FR.get(c['level'], '?')} : {c['message']})"
                    for c in st["components"] if c["level"] > 0]
        level = st["overall_level"]
        if level == 0:
            summary = "Le robot est opérationnel."
        elif level == 1:
            summary = "Le robot est opérationnel en mode dégradé : " + "; ".join(problems) + "."
        else:
            summary = "Le robot signale un composant critique en défaut : " + "; ".join(problems) + "."
        if st["safety_state"] == "EMERGENCY_STOP":
            summary += " Le robot est en arrêt d'urgence."
        nav = ctx.robot.nav_snapshot()
        return ok(summary, operational=level < 2, problems=problems,
                  safety_state=st["safety_state"], motor_status=st["motor_status"],
                  navigation_state=nav.state.value, observations=ctx.robot.get_observations())

    def get_robot_pose() -> dict:
        try:
            pose = ctx.robot.get_pose()
        except PoseUnavailableError:
            return fail("pose_unavailable", "La position du robot n'est pas disponible : "
                        "la localisation n'est pas active.")
        near = ctx.locations.nearest(pose.x, pose.y, 1.5)
        import math
        msg = f"Le robot est en x={pose.x:.2f} m, y={pose.y:.2f} m sur la carte"
        msg += f", près de {near[0].spoken_name}." if near else "."
        return ok(msg, x=round(pose.x, 3), y=round(pose.y, 3),
                  yaw_deg=round(math.degrees(pose.yaw), 1), near=near[0].key if near else None)

    registry.register(Tool("get_robot_status",
                           "État global du robot (santé des composants, sécurité, observations).",
                           _EMPTY, get_robot_status))
    registry.register(Tool("get_robot_pose", "Position actuelle du robot sur la carte.",
                           _EMPTY, get_robot_pose)) 

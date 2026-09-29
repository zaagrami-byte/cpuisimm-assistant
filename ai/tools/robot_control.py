from __future__ import annotations

from typing import TYPE_CHECKING

from assistant.tool_registry import Tool, ToolRegistry, ok
from logsetup import get_logger
from ros2_interface.interface import NavigationError, Nav2UnavailableError, NavState

if TYPE_CHECKING:
    from tools.context import ToolContext

log = get_logger("SAFETY")


def register(registry: ToolRegistry, ctx: "ToolContext") -> None:
    def stop_robot() -> dict:
        """L'arrêt n'est JAMAIS simulé par le dry-run : il reste prioritaire."""
        navigating = False
        try:
            navigating = ctx.robot.cancel_navigation()
        except (Nav2UnavailableError, NavigationError) as exc:
            log.warning("Annulation Nav2 impossible (%s) : on poursuit l'arrêt", exc)
        if ctx.settings.stop_mode == "cancel":
            return ok("J'annule le déplacement en cours." if navigating
                      else "Le robot est déjà à l'arrêt.", stopped=True)
        if not navigating and ctx.robot.is_moving() is False \
                and ctx.robot.nav_snapshot().state != NavState.NAVIGATING:
            return ok("Le robot est déjà à l'arrêt.", stopped=True, already_stopped=True)
        confirmed = ctx.robot.emergency_stop()   # RobotUnavailableError -> message standard
        if confirmed:
            return ok("Arrêt d'urgence activé, le robot est immobilisé. Un opérateur doit le réarmer "
                      "avant tout nouveau déplacement.", stopped=True, estop_confirmed=True)
        return ok("J'ai envoyé l'ordre d'arrêt d'urgence mais le robot ne l'a pas encore confirmé. "
                  "Vérifiez le robot.", stopped=False, estop_confirmed=False)

    registry.register(Tool(
        "stop_robot", "Arrête le robot immédiatement (annule la navigation et déclenche l'arrêt d'urgence).",
        {"type": "object", "properties": {}, "additionalProperties": False}, stop_robot)) 

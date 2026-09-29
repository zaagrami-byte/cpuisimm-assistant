from __future__ import annotations

from typing import TYPE_CHECKING

from assistant.tool_registry import Tool, ToolRegistry, fail, ok
from logsetup import get_logger
from ros2_interface.interface import NavState

if TYPE_CHECKING:
    from tools.context import ToolContext

ai_log = get_logger("AI")


def register(registry: ToolRegistry, ctx: "ToolContext") -> None:
    def navigate_to(location: str) -> dict:
        loc = ctx.locations.resolve(location)
        if loc is None:
            names = ctx.locations.names()
            return fail("unknown_location",
                        "Je ne connais pas cette destination. Voici les destinations disponibles : "
                        + ", ".join(names) + ".", available=names)
        if not loc.calibrated:
            return fail("location_not_calibrated",
                        f"La position de {loc.spoken_name} n'est pas encore enregistrée, "
                        "je ne peux pas y aller.")
        if ctx.settings.dry_run:
            ai_log.info("Tool: navigate_to")
            ai_log.info("Destination: %s", loc.key)
            ai_log.warning("DRY RUN - robot not moved")
            return ok(f"Simulation : le robot irait {loc.spoken_to}, mais le mode test est actif et "
                      "il ne bouge pas.", dry_run=True, destination=loc.key)
        status = ctx.robot.get_status()   # exige un robot joignable
        if status["safety_state"] == "EMERGENCY_STOP":
            return fail("emergency_stop_active", "Le robot est en arrêt d'urgence : un opérateur doit le "
                        "réarmer avant tout déplacement.")
        previous = ctx.robot.nav_snapshot()
        replaced = previous.destination if previous.state == NavState.NAVIGATING else None
        ctx.robot.navigate(loc.key, loc.x, loc.y, loc.yaw_rad)
        message = f"Navigation vers {loc.spoken_name} démarrée."
        if replaced and replaced != loc.key:
            old = ctx.locations.get(replaced)
            message += f" La navigation précédente vers {old.spoken_name if old else replaced} a été annulée."
        return ok(message, dry_run=False, destination=loc.key, replaced=replaced)

    def cancel_navigation() -> dict:
        if ctx.robot.cancel_navigation():
            return ok("La navigation en cours a été annulée.", canceled=True)
        return ok("Aucune navigation n'est en cours.", canceled=False)

    def get_navigation_status() -> dict:
        snap = ctx.robot.nav_snapshot()
        loc = ctx.locations.get(snap.destination)
        to = loc.spoken_to if loc else "à destination"
        name = loc.spoken_name if loc else "la destination"
        messages = {
            NavState.IDLE: "Le robot n'a aucune navigation en cours.",
            NavState.NAVIGATING: f"Le robot se dirige vers {name}."
            + (f" Distance restante : {snap.distance_remaining:.1f} m." if snap.distance_remaining is not None else ""),
            NavState.SUCCEEDED: f"Nav2 a confirmé l'arrivée {to}.",
            NavState.CANCELED: f"La dernière navigation vers {name} a été annulée.",
            NavState.FAILED: f"La dernière navigation vers {name} a échoué. {snap.detail}".strip(),
            NavState.UNKNOWN: "L'état de la navigation est inconnu : Nav2 est injoignable.",
        }
        return ok(messages[snap.state], state=snap.state.value, destination=snap.destination,
                  detail=snap.detail)

    registry.register(Tool(
        "navigate_to",
        "Envoie le robot vers une destination connue (annule automatiquement la navigation précédente).",
        {"type": "object",
         "properties": {"location": {"type": "string", "description": "Nom de la destination, ex: administration"}},
         "required": ["location"], "additionalProperties": False}, navigate_to))
    registry.register(Tool("cancel_navigation", "Annule la navigation en cours.",
                           {"type": "object", "properties": {}, "additionalProperties": False}, cancel_navigation))
    registry.register(Tool("get_navigation_status",
                           "État réel de la navigation (IDLE, NAVIGATING, SUCCEEDED, CANCELED, FAILED, UNKNOWN).",
                           {"type": "object", "properties": {}, "additionalProperties": False}, get_navigation_status)) 

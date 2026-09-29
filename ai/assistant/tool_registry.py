"""Registre d'outils à liste blanche stricte : le LLM ne peut appeler QUE ce qui est enregistré."""
from __future__ import annotations

import json
import re
from dataclasses import dataclass
from typing import Any, Callable

from logsetup import get_logger
from ros2_interface.interface import NavigationError, Nav2UnavailableError, RobotUnavailableError

log = get_logger("TOOL")
safety = get_logger("SAFETY")

MSG_ROBOT_UNREACHABLE = "Je ne peux pas communiquer avec le système de navigation du robot actuellement."
MSG_NAV_UNAVAILABLE = "La navigation n'est pas disponible actuellement."

_NAME_RE = re.compile(r"^[a-z][a-z0-9_]{0,63}$")
_TYPES: dict[str, Any] = {"string": str, "number": (int, float), "integer": int, "boolean": bool}


def ok(message: str, **data: Any) -> dict:
    return {"ok": True, "message": message, **data}


def fail(code: str, message: str, **data: Any) -> dict:
    return {"ok": False, "error": code, "message": message, **data}


class ToolValidationError(Exception): ...


@dataclass(frozen=True)
class Tool:
    name: str
    description: str
    parameters: dict
    handler: Callable[..., dict]


def _validate(schema: dict, args: dict) -> None:
    props = schema.get("properties", {})
    unknown = set(args) - set(props)
    if unknown:
        raise ToolValidationError(f"paramètres inconnus : {sorted(unknown)}")
    for name in schema.get("required", []):
        if name not in args:
            raise ToolValidationError(f"paramètre manquant : {name}")
    for name, value in args.items():
        spec = props[name]
        expected = _TYPES[spec["type"]]
        if (isinstance(value, bool) and spec["type"] != "boolean") or not isinstance(value, expected):
            raise ToolValidationError(f"type invalide pour {name}")
        if spec["type"] == "string":
            if len(value) > 100 or any(ord(c) < 32 for c in value):
                raise ToolValidationError(f"valeur invalide pour {name}")
            if "enum" in spec and value not in spec["enum"]:
                raise ToolValidationError(f"{name} doit être parmi {spec['enum']}")


class ToolRegistry:
    def __init__(self) -> None:
        self._tools: dict[str, Tool] = {}

    def register(self, tool: Tool) -> None:
        if not _NAME_RE.match(tool.name) or tool.name in self._tools:
            raise ValueError(f"nom d'outil invalide ou dupliqué : {tool.name}")
        self._tools[tool.name] = tool

    def names(self) -> list[str]:
        return sorted(self._tools)

    def schemas(self) -> list[dict]:
        return [{"type": "function",
                 "function": {"name": t.name, "description": t.description, "parameters": t.parameters}}
                for t in self._tools.values()]

    def execute(self, name: Any, arguments: Any) -> dict:
        tool = self._tools.get(name) if isinstance(name, str) else None
        if tool is None:
            safety.warning("Outil REFUSÉ (hors liste blanche) : %r args=%r", name, arguments)
            return fail("tool_not_allowed", "Cette action n'est pas autorisée : elle ne fait pas partie "
                        "de mes outils.")
        try:
            if isinstance(arguments, str):
                arguments = json.loads(arguments or "{}")
            if arguments is None:
                arguments = {}
            if not isinstance(arguments, dict):
                raise ToolValidationError("arguments : objet attendu")
            _validate(tool.parameters, arguments)
            log.info("%s(%s)", name, ", ".join(f'{k}="{v}"' for k, v in arguments.items()))
            result = tool.handler(**arguments)
        except (ToolValidationError, ValueError) as exc:
            log.warning("Arguments refusés pour %s : %s", name, exc)
            return fail("invalid_arguments", f"Paramètres invalides : {exc}")
        except RobotUnavailableError as exc:
            log.warning("Robot injoignable (%s)", exc)
            return fail("robot_unreachable", MSG_ROBOT_UNREACHABLE)
        except Nav2UnavailableError as exc:
            log.warning("Nav2 indisponible (%s)", exc)
            return fail("nav2_unavailable", MSG_NAV_UNAVAILABLE)
        except NavigationError as exc:
            log.error("Navigation impossible : %s", exc)
            return fail("navigation_error", f"La navigation a échoué : {exc}")
        except Exception:  # noqa: BLE001
            log.exception("Erreur interne dans l'outil %s", name)
            return fail("internal_error", "Une erreur interne est survenue.")
        log.info("%s -> ok=%s", name, result.get("ok"))
        return result 

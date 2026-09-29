"""Destinations (knowledge/locations.yaml) + outil get_locations. Aucune coordonnée dans le code."""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import yaml

from assistant.tool_registry import Tool, ToolRegistry, ok
from textutil import normalize

if TYPE_CHECKING:
    from tools.context import ToolContext

_ARTICLES = {"le", "la", "les", "l", "au", "aux", "a", "de", "du", "des", "d", "vers", "chez"}


def _strip(norm: str) -> str:
    words = norm.split()
    while words and words[0] in _ARTICLES:
        words.pop(0)
    return " ".join(words)


@dataclass(frozen=True)
class Location:
    key: str
    description: str
    x: float | None
    y: float | None
    yaw_rad: float | None
    aliases: tuple[str, ...]
    spoken_to: str
    spoken_name: str

    @property
    def calibrated(self) -> bool:
        return self.x is not None and self.y is not None and self.yaw_rad is not None


class LocationBook:
    def __init__(self, locations: dict[str, Location]) -> None:
        self._by_key = dict(locations)
        self._index: dict[str, str] = {}
        for loc in locations.values():
            for name in (loc.key, *loc.aliases):
                self._index[_strip(normalize(name))] = loc.key

    @classmethod
    def load(cls, path: Path, expected_frame: str = "map") -> "LocationBook":
        if not path.is_file():
            raise FileNotFoundError(f"locations.yaml introuvable : {path}")
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if data.get("frame", expected_frame) != expected_frame:
            raise ValueError(f"locations.yaml : frame '{data.get('frame')}' != '{expected_frame}'")
        out: dict[str, Location] = {}
        for key, raw in (data.get("locations") or {}).items():
            raw = raw or {}
            x, y, yaw = raw.get("x"), raw.get("y"), raw.get("yaw_deg")
            values = (x, y, yaw)
            if any(v is not None for v in values) and not all(
                    isinstance(v, (int, float)) and math.isfinite(v) for v in values):
                raise ValueError(f"locations.yaml : '{key}' doit avoir x, y, yaw_deg numériques (ou tous null)")
            out[key] = Location(
                key=key, description=str(raw.get("description", key)),
                x=None if x is None else float(x), y=None if y is None else float(y),
                yaw_rad=None if yaw is None else math.radians(float(yaw)),
                aliases=tuple(raw.get("aliases", [])),
                spoken_to=str(raw.get("spoken_to", f"à {key}")),
                spoken_name=str(raw.get("spoken_name", key)))
        return cls(out)

    def names(self) -> list[str]:
        return list(self._by_key)

    def get(self, key: str | None) -> Location | None:
        return self._by_key.get(key) if key else None

    def resolve(self, query: str) -> Location | None:
        """Correspondance EXACTE (après normalisation) : jamais de devinette floue pour une destination."""
        return self.get(self._index.get(_strip(normalize(query))))

    def nearest(self, x: float, y: float, max_dist: float) -> tuple[Location, float] | None:
        best: tuple[Location, float] | None = None
        for loc in self._by_key.values():
            if loc.calibrated:
                d = math.hypot(loc.x - x, loc.y - y)
                if d <= max_dist and (best is None or d < best[1]):
                    best = (loc, d)
        return best


def register(registry: ToolRegistry, ctx: "ToolContext") -> None:
    def get_locations() -> dict:
        items = [{"name": l.key, "description": l.description, "available": l.calibrated}
                 for l in ctx.locations._by_key.values()]
        ready = [i["name"] for i in items if i["available"]]
        message = ("Je peux aller à : " + ", ".join(ready) + "." if ready
                   else "Aucune destination n'est encore enregistrée.")
        return ok(message, locations=items)

    registry.register(Tool(
        "get_locations", "Liste les destinations connues du robot.",
        {"type": "object", "properties": {}, "additionalProperties": False}, get_locations)) 

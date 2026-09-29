"""Base de connaissances ISIMM : YAML -> texte compact injecté dans le prompt."""
from __future__ import annotations

from pathlib import Path

import yaml

from logsetup import get_logger

log = get_logger("AI")
MAX_CHARS = 3000


def _render(obj, indent: int = 0) -> list[str]:
    pad = "  " * indent
    lines: list[str] = []
    if isinstance(obj, dict):
        for key, value in obj.items():
            if isinstance(value, (dict, list)):
                lines.append(f"{pad}{key} :")
                lines.extend(_render(value, indent + 1))
            elif value not in (None, ""):
                lines.append(f"{pad}{key} : {value}")
    elif isinstance(obj, list):
        for item in obj:
            if isinstance(item, (dict, list)):
                lines.append(f"{pad}-")
                lines.extend(_render(item, indent + 1))
            else:
                lines.append(f"{pad}- {item}")
    else:
        lines.append(f"{pad}{obj}")
    return lines


class KnowledgeBase:
    def __init__(self, data: dict) -> None:
        self._data = data

    @classmethod
    def load(cls, path: Path) -> "KnowledgeBase":
        if not path.is_file():
            raise FileNotFoundError(f"Base de connaissances introuvable : {path}")
        return cls(yaml.safe_load(path.read_text(encoding="utf-8")) or {})

    def as_prompt_text(self) -> str:
        text = "\n".join(_render(self._data))
        if len(text) > MAX_CHARS:
            log.warning("Base de connaissances tronquée (%d > %d caractères).", len(text), MAX_CHARS)
            text = text[:MAX_CHARS]
        return text or "(aucune information enregistrée)" 

"""Client Ollama (Qwen) avec tool calling. Ne connaît ni ROS ni l'audio."""
from __future__ import annotations

import json
import re
import time
from dataclasses import dataclass

import httpx
import ollama

from logsetup import get_logger
from settings import Settings

log = get_logger("LLM")


class LLMError(Exception): ...
class LLMUnavailableError(LLMError): ...
class LLMModelNotFoundError(LLMError): ...
class LLMTimeoutError(LLMError): ...


@dataclass(frozen=True)
class ToolCall:
    name: str
    arguments: dict


@dataclass(frozen=True)
class LLMReply:
    content: str
    tool_calls: tuple[ToolCall, ...] = ()

    def as_message(self) -> dict:
        msg: dict = {"role": "assistant", "content": self.content}
        if self.tool_calls:
            msg["tool_calls"] = [{"function": {"name": c.name, "arguments": c.arguments}}
                                 for c in self.tool_calls]
        return msg


def _get(obj, key: str, default=None):
    if obj is None:
        return default
    if isinstance(obj, dict):
        return obj.get(key, default)
    return getattr(obj, key, default)


_JSON_OBJ = re.compile(r"\{.*\}", re.S)


def _text_tool_call(content: str) -> ToolCall | None:
    """Les petits modèles écrivent parfois l'appel d'outil en JSON dans le texte."""
    text = content.strip().removeprefix("<tool_call>").removesuffix("</tool_call>").strip()
    if not text.startswith("{"):
        return None
    match = _JSON_OBJ.search(text)
    if not match:
        return None
    try:
        obj = json.loads(match.group(0))
    except ValueError:
        return None
    name, args = obj.get("name"), obj.get("arguments", {})
    if isinstance(name, str) and isinstance(args, dict):
        return ToolCall(name, args)
    return None


class LLMClient:
    def __init__(self, settings: Settings) -> None:
        self._s = settings
        self._client = ollama.Client(host=settings.ollama_host, timeout=settings.llm_timeout_s)

    def check(self) -> None:
        """Vérifie au démarrage qu'Ollama répond et que le modèle est installé."""
        try:
            listing = self._client.list()
        except (ConnectionError, httpx.TransportError) as exc:
            raise LLMUnavailableError(
                f"Ollama injoignable sur {self._s.ollama_host} ({exc}). Lancer : ollama serve") from exc
        names = {_get(m, "model") or _get(m, "name") for m in (_get(listing, "models") or [])}
        wanted = self._s.ollama_model
        if wanted not in names and f"{wanted}:latest" not in names:
            raise LLMModelNotFoundError(f"Modèle '{wanted}' absent. Lancer : ollama pull {wanted}")
        log.info("Ollama OK, modèle '%s' disponible.", wanted)

    def chat(self, messages: list[dict], tools: list[dict]) -> LLMReply:
        start = time.perf_counter()
        try:
            resp = self._client.chat(
                model=self._s.ollama_model, messages=messages, tools=tools or None,
                options={"temperature": self._s.llm_temperature, "num_ctx": self._s.llm_num_ctx},
                keep_alive=self._s.llm_keep_alive)
        except ollama.ResponseError as exc:
            if exc.status_code == 404:
                raise LLMModelNotFoundError(f"Modèle '{self._s.ollama_model}' introuvable.") from exc
            raise LLMError(f"Erreur Ollama : {exc.error}") from exc
        except httpx.TimeoutException as exc:
            raise LLMTimeoutError(f"Pas de réponse en {self._s.llm_timeout_s:.0f} s.") from exc
        except (ConnectionError, httpx.TransportError) as exc:
            raise LLMUnavailableError(f"Ollama injoignable : {exc}") from exc
        except Exception as exc:  # noqa: BLE001 — le robot ne doit jamais planter ici
            raise LLMError(f"Erreur inattendue : {exc}") from exc

        message = _get(resp, "message")
        content = (_get(message, "content") or "").strip()
        calls: list[ToolCall] = []
        for raw in _get(message, "tool_calls") or []:
            fn = _get(raw, "function")
            name = _get(fn, "name")
            if isinstance(name, str):
                calls.append(ToolCall(name, dict(_get(fn, "arguments") or {})))
        if not calls and content:
            fallback = _text_tool_call(content)
            if fallback:
                calls, content = [fallback], ""
        log.info("Réponse en %.2fs (%d appel(s) d'outil)", time.perf_counter() - start, len(calls))
        return LLMReply(content=content, tool_calls=tuple(calls)) 

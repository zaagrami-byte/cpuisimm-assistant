"""Cerveau conversationnel : chemin rapide sécurité + boucle LLM/outils."""
from __future__ import annotations

import json
import re
from typing import Callable

from assistant.conversation import Conversation
from assistant.knowledge import KnowledgeBase
from assistant.llm_client import LLMClient, LLMError
from assistant.prompts import SYSTEM_PROMPT
from assistant.tool_registry import ToolRegistry
from logsetup import get_logger
from ros2_interface.interface import NavSnapshot, NavState
from settings import Settings
from textutil import normalize
from tools.locations import LocationBook

log = get_logger("AI")
MSG_TECHNICAL = "Je rencontre un petit problème technique, un instant s'il vous plaît."

# Chemin rapide déterministe (avant le LLM) : l'arrêt ne dépend jamais de Qwen.
_STOP_RE = re.compile(r"^(stop|stoppe|stoppe toi|arrete|arrete toi|arret|arret d urgence|halte|"
                      r"urgence|freine)( robot)?( s il (te|vous) plait)?$")
_CANCEL_RE = re.compile(r"\b(annule|arrete|stoppe|interromps|interrompre)\b.*"
                        r"\b(navigation|deplacement|trajet|destination)\b")


class Assistant:
    def __init__(self, settings: Settings, llm: LLMClient, registry: ToolRegistry,
                 conversation: Conversation, knowledge: KnowledgeBase, locations: LocationBook) -> None:
        self._s = settings
        self._llm = llm
        self._registry = registry
        self._conv = conversation
        self._locations = locations
        self._system = SYSTEM_PROMPT.format(
            locations=", ".join(locations.names()) or "(aucune)",
            knowledge=knowledge.as_prompt_text())
        self._announce: Callable[[str], None] | None = None

    def set_announcer(self, fn: Callable[[str], None]) -> None:
        self._announce = fn

    # ---- entrée principale ---------------------------------------------
    def handle(self, user_text: str) -> str:
        text = user_text.strip()
        if not text:
            return ""
        log.info("User: %s", text)
        reply = self._fast_path(text)
        if reply is None:
            reply = self._converse(text)
        self._conv.add_turn(text, reply)
        log.info("Response: %s", reply)
        return reply

    def _fast_path(self, text: str) -> str | None:
        norm = normalize(text)
        if _STOP_RE.match(norm):
            log.warning("Commande d'arrêt reconnue -> stop_robot (sans LLM)")
            return self._registry.execute("stop_robot", {})["message"]
        if _CANCEL_RE.search(norm):
            log.info("Annulation reconnue -> cancel_navigation (sans LLM)")
            return self._registry.execute("cancel_navigation", {})["message"]
        return None

    def _converse(self, text: str) -> str:
        messages: list[dict] = [{"role": "system", "content": self._system},
                                *self._conv.messages(), {"role": "user", "content": text}]
        tools = self._registry.schemas()
        last_message: str | None = None
        for _ in range(self._s.max_tool_rounds):
            log.info("Processing request")
            try:
                reply = self._llm.chat(messages, tools)
            except LLMError as exc:
                log.error("LLM indisponible : %s", exc)
                return MSG_TECHNICAL
            if not reply.tool_calls:
                return reply.content or last_message or "Je n'ai pas pu traiter votre demande."
            messages.append(reply.as_message())
            for call in reply.tool_calls[:3]:
                result = self._registry.execute(call.name, call.arguments)
                last_message = result.get("message", last_message)
                messages.append({"role": "tool", "content": json.dumps(result, ensure_ascii=False)})
        return last_message or "Je n'ai pas pu terminer cette demande."

    # ---- événements Nav2 -> annonces déterministes ----------------------
    def on_nav_event(self, snap: NavSnapshot) -> None:
        loc = self._locations.get(snap.destination) if snap.destination else None
        to = loc.spoken_to if loc else "à destination"
        name = loc.spoken_name if loc else "la destination"
        if snap.state == NavState.SUCCEEDED:
            text = f"Je suis arrivé {to}."
        elif snap.state == NavState.FAILED:
            text = f"La navigation vers {name} a échoué."
        elif snap.state == NavState.CANCELED:
            text = f"La navigation vers {name} a été annulée."
        else:
            return
        log.info("Annonce : %s", text)
        if self._announce:
            self._announce(text) 

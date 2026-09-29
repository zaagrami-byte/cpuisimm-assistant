from __future__ import annotations

from collections import deque


class Conversation:
    """Mémoire courte : `max_turns` échanges (utilisateur + réponse finale)."""

    def __init__(self, max_turns: int) -> None:
        self._turns: deque[tuple[str, str]] = deque(maxlen=max(1, max_turns))

    def add_turn(self, user: str, assistant: str) -> None:
        self._turns.append((user, assistant))

    def messages(self) -> list[dict]:
        out: list[dict] = []
        for user, assistant in self._turns:
            out.append({"role": "user", "content": user})
            out.append({"role": "assistant", "content": assistant})
        return out

    def clear(self) -> None:
        self._turns.clear() 

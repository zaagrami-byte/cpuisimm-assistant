"""Boucle vocale : LISTENING -> TRANSCRIBING -> THINKING -> SPEAKING -> LISTENING."""
from __future__ import annotations

import queue
import re
import threading
import time

from assistant.assistant import Assistant
from audio.server import AudioError, AudioNotConnectedError, AudioServer
from audio.vad import UtteranceSegmenter
from logsetup import get_logger
from speech.stt import STTError, WhisperSTT
from speech.tts import PiperTTS, TTSError
from textutil import normalize

log = get_logger("AI")
_MD = re.compile(r"[*#`_>]+")


class WakePolicy:
    """Wake word par transcription (aucune dépendance) : « Bonjour robot, va à l'accueil »."""

    def __init__(self, enabled: bool, phrases: tuple[str, ...], session_s: float) -> None:
        self._enabled = enabled
        self._phrases = [normalize(p).split() for p in phrases if normalize(p)]
        self._session_s = session_s
        self._until = 0.0

    def filter(self, text: str) -> str | None:
        if not self._enabled:
            return text
        now = time.monotonic()
        if now < self._until:
            self._until = now + self._session_s
            return text
        words = text.split()
        norm = [normalize(w) for w in words]
        for phrase in self._phrases:
            n = len(phrase)
            if [w for w in norm[:n]] == phrase:
                self._until = now + self._session_s
                rest = " ".join(words[n:]).strip(" ,.!?;:")
                return rest or "Bonjour"
        log.info("Wake word absent, phrase ignorée.")
        return None


class VoiceLoop:
    def __init__(self, server: AudioServer, stt: WhisperSTT, tts: PiperTTS, assistant: Assistant,
                 segmenter: UtteranceSegmenter, wake: WakePolicy) -> None:
        self._server, self._stt, self._tts = server, stt, tts
        self._assistant, self._segmenter, self._wake = assistant, segmenter, wake
        self._announcements: "queue.Queue[str]" = queue.Queue()
        self._was_connected = False

    def announce(self, text: str) -> None:
        self._announcements.put(text)

    def run(self, stop: threading.Event) -> None:
        while not stop.is_set():
            try:
                self._step()
            except Exception:  # noqa: BLE001 — la boucle ne doit jamais mourir
                log.exception("Erreur inattendue dans la boucle vocale")
                time.sleep(1.0)

    def _step(self) -> None:
        connected = self._server.connected
        if connected != self._was_connected:
            self._was_connected = connected
            log.info("Robot %s.", "connecté" if connected else "déconnecté")
        if not connected:
            time.sleep(0.5)
            return
        try:
            text = self._announcements.get_nowait()
        except queue.Empty:
            pass
        else:
            self.speak(text)
            return
        frame = self._server.read_mic(timeout=0.2)
        if frame is None:
            return
        utterance = self._segmenter.feed(frame)
        if utterance is None:
            return
        log.info("Transcribing")
        try:
            text = self._stt.transcribe(utterance)
        except STTError as exc:
            log.error("STT : %s", exc)
            return
        if not text:
            return
        gated = self._wake.filter(text)
        if gated is None:
            return
        self.speak(self._assistant.handle(gated))

    def speak(self, text: str) -> None:
        text = re.sub(r"\s+", " ", _MD.sub("", text)).strip()
        if not text:
            return
        log.info("Speaking")
        try:
            self._server.clear_mic()
            self._server.play(self._tts.synthesize(text))
        except AudioNotConnectedError:
            log.warning("Réponse non prononcée : robot non connecté.")
        except (TTSError, AudioError) as exc:
            log.error("Réponse non prononcée : %s", exc)
        finally:
            self._server.clear_mic()
            self._segmenter.reset() 

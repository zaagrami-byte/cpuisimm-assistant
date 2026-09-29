"""Piper : synthèse phrase par phrase (le robot commence à parler avant la fin de la génération)."""
from __future__ import annotations

import io
import re
import time
import wave
from typing import Iterator

from logsetup import get_logger
from settings import Settings

log = get_logger("TTS")
_SENTENCES = re.compile(r"(?<=[.!?…])\s+")


class TTSError(Exception): ...


def split_sentences(text: str) -> list[str]:
    return [p.strip() for p in _SENTENCES.split(text) if p.strip()]


class PiperTTS:
    def __init__(self, settings: Settings) -> None:
        self._model_path = settings.piper_model
        self._voice = None

    def load(self) -> None:
        config_path = self._model_path.with_name(self._model_path.name + ".json")
        if not self._model_path.is_file() or not config_path.is_file():
            raise TTSError(f"Modèle Piper introuvable : {self._model_path} (+ .json)")
        try:
            from piper.voice import PiperVoice
            start = time.perf_counter()
            self._voice = PiperVoice.load(str(self._model_path), str(config_path))
        except Exception as exc:  # noqa: BLE001
            raise TTSError(f"Chargement de Piper impossible : {exc}") from exc
        log.info("Voix Piper chargée en %.1f s (%s).", time.perf_counter() - start, self._model_path.name)

    def synthesize(self, text: str) -> Iterator[tuple[int, bytes]]:
        """Génère (sample_rate, PCM s16le mono) une phrase à la fois."""
        if self._voice is None:
            self.load()
        for sentence in split_sentences(text):
            start = time.perf_counter()
            log.info("Generating audio")
            try:
                buf = io.BytesIO()
                with wave.open(buf, "wb") as out:
                    self._voice.synthesize_wav(sentence, out)
                buf.seek(0)
                with wave.open(buf, "rb") as wav:
                    if wav.getnchannels() != 1 or wav.getsampwidth() != 2:
                        raise TTSError("format Piper inattendu (mono 16 bits attendu)")
                    rate, pcm = wav.getframerate(), wav.readframes(wav.getnframes())
            except TTSError:
                raise
            except Exception as exc:  # noqa: BLE001
                raise TTSError(f"Échec de synthèse : {exc}") from exc
            log.info("Phrase synthétisée en %.2fs", time.perf_counter() - start)
            yield rate, pcm 

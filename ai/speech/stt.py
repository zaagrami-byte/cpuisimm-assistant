"""faster-whisper (CPU/int8). Reçoit du PCM s16le 16 kHz mono."""
from __future__ import annotations

import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeout

import numpy as np

from logsetup import get_logger
from settings import Settings
from textutil import normalize

log = get_logger("STT")


class STTError(Exception): ...


class WhisperSTT:
    def __init__(self, settings: Settings) -> None:
        self._s = settings
        self._model = None
        self._pool = ThreadPoolExecutor(max_workers=1)
        self._ignored = {normalize(p) for p in settings.stt_ignore_phrases}

    def load(self) -> None:
        try:
            from faster_whisper import WhisperModel
            start = time.perf_counter()
            self._s.stt_model_dir.mkdir(parents=True, exist_ok=True)
            self._model = WhisperModel(self._s.whisper_model, device=self._s.whisper_device,
                                       compute_type=self._s.whisper_compute_type,
                                       download_root=str(self._s.stt_model_dir))
        except Exception as exc:  # noqa: BLE001
            raise STTError(f"Chargement de Whisper impossible : {exc}") from exc
        log.info("Whisper '%s' chargé en %.1f s.", self._s.whisper_model, time.perf_counter() - start)

    def transcribe(self, pcm: bytes) -> str:
        """Texte reconnu, ou '' si rien d'exploitable (silence/hallucination filtrée)."""
        if self._model is None:
            self.load()
        if not pcm:
            return ""
        audio = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
        peak = float(np.max(np.abs(audio))) if audio.size else 0.0
        if 0.005 < peak < 0.3:  # micro faible : pic ramené à ~0.5 (gain max x30)
            audio = np.clip(audio * min(0.5 / peak, 30.0), -1.0, 1.0)
        start = time.perf_counter()
        future = self._pool.submit(self._run, audio)
        try:
            text = future.result(timeout=self._s.stt_timeout_s)
        except FutureTimeout as exc:
            raise STTError(f"Transcription > {self._s.stt_timeout_s:.0f} s") from exc
        except Exception as exc:  # noqa: BLE001
            raise STTError(f"Échec de transcription : {exc}") from exc
        if normalize(text) in self._ignored:
            log.info("Hallucination Whisper ignorée : %r", text)
            return ""
        log.info("User: %s  (%.2fs pour %.1fs d'audio)", text, time.perf_counter() - start,
                 len(audio) / 16000)
        return text

    def _run(self, audio: np.ndarray) -> str:
        segments, _ = self._model.transcribe(
            audio, language=self._s.language, beam_size=self._s.whisper_beam_size,
            vad_filter=True, vad_parameters={"min_silence_duration_ms": 500, "threshold": 0.35},
            no_speech_threshold=0.6, log_prob_threshold=-1.0,
            compression_ratio_threshold=2.4, condition_on_previous_text=False)
        return " ".join(seg.text.strip() for seg in segments).strip() 

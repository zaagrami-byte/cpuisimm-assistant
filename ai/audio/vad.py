"""Découpe le flux micro en énoncés (webrtcvad injecté -> testable sans la lib)."""
from __future__ import annotations

import collections
from typing import Callable


class UtteranceSegmenter:
    def __init__(self, is_speech: Callable[[bytes], bool], sample_rate: int, frame_ms: int,
                 start_window_ms: int, start_ratio: float, end_silence_ms: int, end_ratio: float,
                 max_utterance_s: float, min_utterance_ms: int) -> None:
        self._is_speech = is_speech
        self._frame_bytes = sample_rate * frame_ms // 1000 * 2
        self._start_ratio, self._end_ratio = start_ratio, end_ratio
        self._start_n = max(1, round(start_window_ms / frame_ms))
        self._end_n = max(1, round(end_silence_ms / frame_ms))
        self._max_frames = int(max_utterance_s * 1000 / frame_ms)
        self._min_frames = max(1, int(min_utterance_ms / frame_ms))
        self._leftover = b""
        self.reset()

    def reset(self) -> None:
        self._pre: collections.deque = collections.deque(maxlen=self._start_n)
        self._end: collections.deque = collections.deque(maxlen=self._end_n)
        self._frames: list[bytes] = []
        self._triggered = False
        self._leftover = b""

    def feed(self, data: bytes) -> bytes | None:
        """Renvoie un énoncé complet (PCM s16le) dès que la fin de phrase est détectée."""
        data = self._leftover + data
        n = self._frame_bytes
        pos = 0
        while len(data) - pos >= n:
            frame = data[pos:pos + n]
            pos += n
            utterance = self._feed_frame(frame)
            if utterance is not None:
                self.reset()
                return utterance if len(utterance) // n >= self._min_frames else None
        self._leftover = data[pos:]
        return None

    def _feed_frame(self, frame: bytes) -> bytes | None:
        speech = self._is_speech(frame)
        if not self._triggered:
            self._pre.append((frame, speech))
            if sum(1 for _, s in self._pre if s) >= self._start_ratio * self._pre.maxlen:
                self._triggered = True
                self._frames = [f for f, _ in self._pre]  # garde le début de mot
                self._pre.clear()
            return None
        self._frames.append(frame)
        self._end.append(speech)
        if len(self._end) == self._end.maxlen:
            silent = sum(1 for s in self._end if not s)
            if silent >= self._end_ratio * self._end.maxlen:
                return b"".join(self._frames)
        if len(self._frames) >= self._max_frames:
            return b"".join(self._frames)
        return None 

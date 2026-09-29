"""Tests 4 et 5 sans robot.

  python -m scripts.speech_probe stt fichier.wav          # WAV mono 16 kHz
  python -m scripts.speech_probe tts "Bonjour" --out /tmp/t.wav
"""
from __future__ import annotations

import argparse
import sys
import time
import wave

from logsetup import setup_logging
from settings import load_settings
from speech.stt import WhisperSTT
from speech.tts import PiperTTS


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p1 = sub.add_parser("stt")
    p1.add_argument("wav")
    p2 = sub.add_parser("tts")
    p2.add_argument("text")
    p2.add_argument("--out", default="/tmp/isimm_tts.wav")
    a = ap.parse_args()
    s = load_settings()
    setup_logging(s.log_level, s.log_dir)
    if a.cmd == "stt":
        with wave.open(a.wav) as w:
            if (w.getframerate(), w.getnchannels(), w.getsampwidth()) != (16000, 1, 2):
                print("Le WAV doit être mono, 16 kHz, 16 bits.")
                return 2
            pcm = w.readframes(w.getnframes())
        stt = WhisperSTT(s)
        stt.load()
        print(f"Texte : {stt.transcribe(pcm)!r}")
    else:
        tts = PiperTTS(s)
        tts.load()
        start = time.perf_counter()
        rate, data = 0, b""
        for rate, pcm in tts.synthesize(a.text):
            data += pcm
        with wave.open(a.out, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(rate)
            w.writeframes(data)
        print(f"{a.out} : {len(data) / (2 * rate):.1f} s d'audio, généré en {time.perf_counter() - start:.2f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main()) 

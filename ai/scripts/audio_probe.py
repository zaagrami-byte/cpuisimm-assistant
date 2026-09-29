"""Tests 2 et 3 : liaison audio Pi <-> PC (sans IA).

  python -m scripts.audio_probe record --seconds 5 --out /tmp/robot_mic.wav   # Pi -> PC
  python -m scripts.audio_probe tone                                          # PC -> Pi (bip)
  python -m scripts.audio_probe speak "Bonjour, je suis le robot."           # PC -> Pi (Piper)
"""
from __future__ import annotations

import argparse
import array
import math
import sys
import time
import wave

from audio.server import AudioServer
from logsetup import setup_logging
from settings import load_settings


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("--seconds", type=float, default=5)
    r.add_argument("--out", default="/tmp/robot_mic.wav")
    sub.add_parser("tone")
    sp = sub.add_parser("speak")
    sp.add_argument("text")
    a = ap.parse_args()
    s = load_settings()
    setup_logging(s.log_level, s.log_dir)
    server = AudioServer(s)
    server.start()
    try:
        print("En attente du robot (lancer 'python3 -m robot_audio' sur le Pi)...")
        if not server.wait_connected(60):
            print("Aucun robot connecté en 60 s.")
            return 1
        if a.cmd == "record":
            print(f"Parlez près du robot pendant {a.seconds:.0f} s...")
            server.clear_mic()
            frames, end = [], time.monotonic() + a.seconds
            while time.monotonic() < end:
                f = server.read_mic(0.5)
                if f:
                    frames.append(f)
            data = b"".join(frames)
            with wave.open(a.out, "wb") as w:
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(s.audio_sample_rate)
                w.writeframes(data)
            samples = array.array("h")
            samples.frombytes(data)
            peak = max((abs(x) for x in samples), default=0)
            rms = math.sqrt(sum(x * x for x in samples) / len(samples)) if samples else 0
            print(f"{a.out} : {len(data) / (2 * s.audio_sample_rate):.1f} s reçues (attendu ~{a.seconds:.0f}), "
                  f"crête={peak}, RMS={rms:.0f}")
        elif a.cmd == "tone":
            rate = 22050
            tone = array.array("h", (int(9000 * math.sin(2 * math.pi * 440 * i / rate)) for i in range(rate * 2)))
            server.play([(rate, tone.tobytes())])
            print("Bip envoyé et lecture confirmée par le robot.")
        else:
            from speech.tts import PiperTTS
            tts = PiperTTS(s)
            tts.load()
            server.play(tts.synthesize(a.text))
            print("Phrase jouée et confirmée par le robot.")
    finally:
        server.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main()) 

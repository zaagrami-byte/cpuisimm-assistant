"""Client audio du robot (Raspberry Pi 4) — AUCUN modèle IA, stdlib uniquement.

  micro (arecord) ──TCP──► PC        PC ──TCP──► haut-parleur (aplay)

Usage (depuis la racine du dépôt) :
  python3 -m robot_audio                  # service normal
  python3 -m robot_audio --list-devices   # arecord -l / aplay -l
  python3 -m robot_audio --test-mic       # TEST 1 : niveau du micro
  python3 -m robot_audio --test-speaker   # bip 440 Hz
Configuration : variables d'environnement AUDIO_* (voir /etc/isimm/audio.env).
"""
from __future__ import annotations

import argparse
import array
import logging
import math
import os
import queue
import shlex
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import wave
from dataclasses import dataclass

from . import protocol as P

_FMT = "%(asctime)s %(levelname)-7s [%(name)s] %(message)s"
log = logging.getLogger("AUDIO")
net = logging.getLogger("NETWORK")
SHUTDOWN = threading.Event()


@dataclass(frozen=True)
class Config:
    pc_host: str
    port: int
    capture_device: str
    playback_device: str
    mic_rate: int
    frame_ms: int
    mute_tail_s: float
    ping_interval: float
    peer_timeout: float
    capture_extra: tuple[str, ...]
    playback_extra: tuple[str, ...]
    robot_id: str

    @property
    def frame_bytes(self) -> int:
        return self.mic_rate * self.frame_ms // 1000 * 2

    @classmethod
    def from_env(cls) -> "Config":
        e = os.environ.get
        return cls(
            pc_host=e("AUDIO_PC_HOST", ""),
            port=int(e("AUDIO_PORT", "5005")),
            capture_device=e("AUDIO_CAPTURE_DEVICE", "default"),
            playback_device=e("AUDIO_PLAYBACK_DEVICE", "default"),
            mic_rate=int(e("AUDIO_MIC_RATE", "16000")),
            frame_ms=int(e("AUDIO_FRAME_MS", "20")),
            mute_tail_s=int(e("AUDIO_MUTE_TAIL_MS", "400")) / 1000.0,
            ping_interval=float(e("AUDIO_PING_INTERVAL_S", "2")),
            peer_timeout=float(e("AUDIO_PEER_TIMEOUT_S", "6")),
            capture_extra=tuple(shlex.split(e("AUDIO_CAPTURE_EXTRA", ""))),
            playback_extra=tuple(shlex.split(e("AUDIO_PLAYBACK_EXTRA", ""))),
            robot_id=e("AUDIO_ROBOT_ID", "isimm-robot"),
        )


def _read_exact(stream, n: int) -> bytes | None:
    buf = bytearray()
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


def _pump_stderr(proc: subprocess.Popen, name: str) -> None:
    for raw in iter(proc.stderr.readline, b""):
        line = raw.decode("utf-8", "replace").strip()
        if line:
            log.warning("%s : %s", name, line)


class Session:
    """Une connexion active avec le PC (threads : micro, lecture, heartbeat, réception)."""

    def __init__(self, cfg: Config, sock: socket.socket, reader: P.FrameReader) -> None:
        self.cfg = cfg
        self.sock = sock
        self.reader = reader
        self.send_lock = threading.Lock()
        self.stop = threading.Event()
        self.speaking = threading.Event()
        self.muted_until = 0.0
        self.last_rx = time.monotonic()
        self.play_q: "queue.Queue[tuple[str, object]]" = queue.Queue()
        self._aplay: subprocess.Popen | None = None
        self._aplay_lock = threading.Lock()
        self._capture: subprocess.Popen | None = None
        self._threads: list[threading.Thread] = []

    def stopped(self) -> bool:
        return self.stop.is_set() or SHUTDOWN.is_set()

    def send(self, msg_type: P.MsgType, payload: bytes = b"") -> None:
        try:
            with self.send_lock:
                P.send_all(self.sock, P.pack(msg_type, payload))
        except (OSError, P.ConnectionClosed):
            self.stop.set()
            raise

    def run(self) -> None:
        for target, name in ((self._mic_loop, "mic"), (self._play_loop, "play"),
                             (self._heartbeat, "heartbeat")):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)
        try:
            self._rx_loop()
        finally:
            self._shutdown()

    # ---- réception -------------------------------------------------
    def _rx_loop(self) -> None:
        while not self.stopped():
            try:
                msg_type, payload = self.reader.read()
            except socket.timeout:
                continue
            self.last_rx = time.monotonic()
            if msg_type == P.MsgType.PONG:
                continue
            if msg_type == P.MsgType.PING:
                self.send(P.MsgType.PONG, payload)
            elif msg_type == P.MsgType.SPK_BEGIN:
                self.speaking.set()
                self.play_q.put(("begin", P.unpack_json(payload)))
            elif msg_type == P.MsgType.SPK_DATA:
                self.play_q.put(("data", payload))
            elif msg_type == P.MsgType.SPK_END:
                self.play_q.put(("end", P.unpack_json(payload)))
            elif msg_type == P.MsgType.SPK_CANCEL:
                self._cancel_playback()
            elif msg_type == P.MsgType.BYE:
                net.info("Le PC a fermé la session proprement.")
                return

    def _cancel_playback(self) -> None:
        log.info("Lecture interrompue par le PC.")
        self._kill_aplay()
        with self.play_q.mutex:
            self.play_q.queue.clear()
        self.play_q.put(("cancel", None))
        self.muted_until = time.monotonic() + self.cfg.mute_tail_s
        self.speaking.clear()

    # ---- micro -----------------------------------------------------
    def _mic_loop(self) -> None:
        cfg = self.cfg
        cmd = ["arecord", "-D", cfg.capture_device, "-f", "S16_LE", "-r", str(cfg.mic_rate),
               "-c", "1", "-t", "raw", "-q", *cfg.capture_extra]
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        except OSError as exc:
            log.error("Impossible de lancer arecord : %s (apt install alsa-utils)", exc)
            self.stop.set()
            return
        self._capture = proc
        threading.Thread(target=_pump_stderr, args=(proc, "arecord"), daemon=True).start()
        log.info("Capture démarrée (%s, %d Hz, trames de %d ms).",
                 cfg.capture_device, cfg.mic_rate, cfg.frame_ms)
        try:
            while not self.stopped():
                frame = _read_exact(proc.stdout, cfg.frame_bytes)
                if frame is None:
                    if not self.stopped():
                        log.error("arecord s'est arrêté (périphérique '%s' indisponible ?).",
                                  cfg.capture_device)
                    break
                if self.speaking.is_set() or time.monotonic() < self.muted_until:
                    continue  # micro coupé pendant la parole du robot
                self.send(P.MsgType.MIC, frame)
        except (OSError, P.ConnectionClosed):
            pass
        finally:
            self.stop.set()

    # ---- lecture ---------------------------------------------------
    def _start_aplay(self, rate: int, channels: int) -> subprocess.Popen | None:
        cmd = ["aplay", "-D", self.cfg.playback_device, "-t", "raw", "-f", "S16_LE",
               "-r", str(rate), "-c", str(channels), "-q", *self.cfg.playback_extra, "-"]
        try:
            proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        except OSError as exc:
            log.error("Impossible de lancer aplay : %s", exc)
            return None
        threading.Thread(target=_pump_stderr, args=(proc, "aplay"), daemon=True).start()
        with self._aplay_lock:
            self._aplay = proc
        return proc

    def _kill_aplay(self) -> None:
        with self._aplay_lock:
            proc, self._aplay = self._aplay, None
        if proc is not None and proc.poll() is None:
            proc.kill()
            try:
                proc.wait(1)
            except subprocess.TimeoutExpired:
                pass

    def _play_loop(self) -> None:
        proc: subprocess.Popen | None = None
        utterance_id = 0
        while not self.stopped():
            try:
                kind, data = self.play_q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if kind == "begin":
                    self._kill_aplay()
                    utterance_id = int(data.get("utterance_id", 0))
                    proc = self._start_aplay(int(data["rate"]), int(data.get("channels", 1)))
                elif kind == "data":
                    if proc is not None and proc.poll() is None:
                        proc.stdin.write(data)  # bloquant = contre-pression vers le PC
                elif kind == "end":
                    if proc is not None:
                        proc.stdin.close()
                        proc.wait(timeout=120)
                    proc = None
                    self._playback_finished(utterance_id)
                elif kind == "cancel":
                    proc = None
            except (BrokenPipeError, ValueError, OSError, KeyError, subprocess.TimeoutExpired) as exc:
                log.warning("Lecture interrompue : %s", exc)
                self._kill_aplay()
                if kind in ("data", "begin"):
                    proc = None
            except P.ConnectionClosed:
                break

    def _playback_finished(self, utterance_id: int) -> None:
        self.muted_until = time.monotonic() + self.cfg.mute_tail_s
        self.speaking.clear()
        self.send(P.MsgType.SPK_DONE, P.pack_json(P.MsgType.SPK_DONE, {"utterance_id": utterance_id})[5:])

    # ---- heartbeat -------------------------------------------------
    def _heartbeat(self) -> None:
        while not self.stop.wait(self.cfg.ping_interval):
            if SHUTDOWN.is_set():
                self.stop.set()
                break
            silent = time.monotonic() - self.last_rx
            if silent > self.cfg.peer_timeout:
                net.warning("PC silencieux depuis %.0f s : session abandonnée.", silent)
                self.stop.set()
                break
            try:
                self.send(P.MsgType.PING, struct.pack("!d", time.time()))
            except (OSError, P.ConnectionClosed):
                break

    def _shutdown(self) -> None:
        self.stop.set()
        try:
            self.send(P.MsgType.BYE)
        except Exception:
            pass
        self._kill_aplay()
        if self._capture is not None and self._capture.poll() is None:
            self._capture.kill()
        try:
            self.sock.close()
        except OSError:
            pass
        for t in self._threads:
            t.join(timeout=1.0)


def connect(cfg: Config) -> tuple[socket.socket, P.FrameReader]:
    sock = socket.create_connection((cfg.pc_host, cfg.port), timeout=5.0)
    try:
        P.tune_socket(sock)
        sock.settimeout(1.0)
        hello = {"version": P.VERSION, "robot_id": cfg.robot_id, "mic_rate": cfg.mic_rate,
                 "mic_channels": 1, "frame_ms": cfg.frame_ms}
        P.send_all(sock, P.pack_json(P.MsgType.HELLO, hello))
        reader = P.FrameReader(sock)
        deadline = time.monotonic() + 5.0
        while True:
            try:
                msg_type, payload = reader.read()
                break
            except socket.timeout:
                if time.monotonic() > deadline:
                    raise TimeoutError("HELLO_ACK non reçu")
        if msg_type != P.MsgType.HELLO_ACK:
            raise P.ProtocolError(f"HELLO_ACK attendu, reçu {msg_type.name}")
        ack = P.unpack_json(payload)
        if ack.get("version") != P.VERSION:
            raise P.ProtocolError(f"version protocole incompatible : {ack}")
        return sock, reader
    except Exception:
        sock.close()
        raise


def run_forever(cfg: Config) -> int:
    net.info("Cible : %s:%d", cfg.pc_host, cfg.port)
    backoff = 1.0
    while not SHUTDOWN.is_set():
        try:
            sock, reader = connect(cfg)
            net.info("Connecté au PC %s:%d.", cfg.pc_host, cfg.port)
            backoff = 1.0
            Session(cfg, sock, reader).run()
            net.info("Session terminée.")
        except (OSError, P.ProtocolError, P.ConnectionClosed, TimeoutError) as exc:
            net.warning("Liaison PC indisponible (%s). Nouvel essai dans %.0f s.", exc, backoff)
        if SHUTDOWN.wait(backoff):
            break
        backoff = min(backoff * 2, 5.0)
    log.info("Arrêt propre du client audio.")
    return 0


# ---------------------------------------------------------------- outils de test
def list_devices() -> int:
    for cmd in (["arecord", "-l"], ["aplay", "-l"]):
        print(f"$ {' '.join(cmd)}")
        if shutil.which(cmd[0]) is None:
            print(f"  {cmd[0]} introuvable : sudo apt install alsa-utils\n")
            continue
        subprocess.run(cmd, check=False)
        print()
    print("Choisir un périphérique : 'carte X, périphérique Y' -> AUDIO_CAPTURE_DEVICE=plughw:X,Y "
          "(idem AUDIO_PLAYBACK_DEVICE). Test : arecord -D plughw:X,Y -d 3 -f S16_LE -r 16000 /tmp/t.wav")
    return 0


def test_mic(cfg: Config, seconds: int) -> int:
    out = "/tmp/isimm_mic_test.wav"
    print(f"Parlez pendant {seconds} s (périphérique {cfg.capture_device})...")
    try:
        subprocess.run(["arecord", "-D", cfg.capture_device, "-f", "S16_LE", "-r", str(cfg.mic_rate),
                        "-c", "1", "-d", str(seconds), "-t", "wav", "-q", out], check=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"ÉCHEC de l'enregistrement : {exc}")
        return 1
    with wave.open(out) as w:
        samples = array.array("h")
        samples.frombytes(w.readframes(w.getnframes()))
    if not samples:
        print("Aucun échantillon enregistré.")
        return 1
    peak = max(abs(s) for s in samples)
    rms = math.sqrt(sum(s * s for s in samples) / len(samples))
    print(f"Fichier : {out} | crête={peak}/32767 | RMS={rms:.0f}")
    if peak < 300:
        print("=> Signal quasi nul : mauvais périphérique, micro coupé ou gain à zéro (alsamixer).")
        return 1
    if peak > 32000:
        print("=> Saturation : baisser le gain (alsamixer).")
    else:
        print("=> Niveau correct.")
    return 0


def test_speaker(cfg: Config) -> int:
    rate = 22050
    tone = array.array("h", (int(9000 * math.sin(2 * math.pi * 440 * i / rate)) for i in range(rate * 2)))
    print(f"Bip 440 Hz de 2 s sur {cfg.playback_device}...")
    try:
        subprocess.run(["aplay", "-D", cfg.playback_device, "-t", "raw", "-f", "S16_LE", "-r", str(rate),
                        "-c", "1", "-q", "-"], input=tone.tobytes(), check=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"ÉCHEC de lecture : {exc}")
        return 1
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="robot_audio", description=__doc__.splitlines()[0])
    ap.add_argument("--list-devices", action="store_true")
    ap.add_argument("--test-mic", action="store_true")
    ap.add_argument("--test-speaker", action="store_true")
    ap.add_argument("--seconds", type=int, default=5)
    args = ap.parse_args(argv)
    logging.basicConfig(level=os.environ.get("AUDIO_LOG_LEVEL", "INFO"), format=_FMT)
    cfg = Config.from_env()
    if args.list_devices:
        return list_devices()
    if args.test_mic:
        return test_mic(cfg, args.seconds)
    if args.test_speaker:
        return test_speaker(cfg)
    if not cfg.pc_host:
        print("AUDIO_PC_HOST n'est pas défini (voir /etc/isimm/audio.env).", file=sys.stderr)
        return 2
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: SHUTDOWN.set())
    return run_forever(cfg) 

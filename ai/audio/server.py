"""Serveur audio côté PC : accepte le robot, reçoit le micro, envoie la voix Piper."""
from __future__ import annotations

import queue
import socket
import threading
import time
from typing import Iterable

from audio import protocol as P
from logsetup import get_logger
from settings import Settings

log = get_logger("AUDIO")
net = get_logger("NETWORK")
_CHUNK = 4096


class AudioError(Exception): ...
class AudioNotConnectedError(AudioError): ...
class AudioConnectionLostError(AudioError): ...
class AudioTimeoutError(AudioError): ...


class _Session:
    def __init__(self, server: "AudioServer", conn: socket.socket, reader: P.FrameReader,
                 addr, hello: dict) -> None:
        self._server, self._conn, self._reader = server, conn, reader
        self.addr, self.hello = addr, hello
        self._send_lock = threading.Lock()
        self._pending: dict[int, threading.Event] = {}
        self._plock = threading.Lock()
        self.closed = threading.Event()
        self.last_rx = time.monotonic()
        self._thread = threading.Thread(target=self._run, name="audio-session", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def send(self, msg_type: P.MsgType, payload: bytes = b"") -> None:
        if self.closed.is_set():
            raise AudioConnectionLostError("session fermée")
        try:
            with self._send_lock:
                P.send_all(self._conn, P.pack(msg_type, payload))
        except (OSError, P.ConnectionClosed) as exc:
            self.close(f"envoi impossible ({exc})")
            raise AudioConnectionLostError(str(exc)) from exc

    def expect_done(self, utterance_id: int) -> threading.Event:
        ev = threading.Event()
        with self._plock:
            self._pending[utterance_id] = ev
        return ev

    def close(self, reason: str) -> None:
        if self.closed.is_set():
            return
        self.closed.set()
        net.info("Session robot %s fermée : %s", self.addr[0], reason)
        try:
            self._conn.close()
        except OSError:
            pass
        with self._plock:
            for ev in self._pending.values():
                ev.set()  # débloque play() : il verra closed
        self._server._on_session_end(self)

    def _run(self) -> None:
        s = self._server._s
        try:
            while not self.closed.is_set():
                try:
                    msg_type, payload = self._reader.read()
                except socket.timeout:
                    if time.monotonic() - self.last_rx > s.audio_peer_timeout_s:
                        self.close("robot silencieux (timeout)")
                    continue
                self.last_rx = time.monotonic()
                if msg_type == P.MsgType.MIC:
                    self._server._push_mic(payload)
                elif msg_type == P.MsgType.PING:
                    self.send(P.MsgType.PONG, payload)
                elif msg_type == P.MsgType.SPK_DONE:
                    uid = int(P.unpack_json(payload).get("utterance_id", -1))
                    with self._plock:
                        ev = self._pending.pop(uid, None)
                    if ev:
                        ev.set()
                elif msg_type == P.MsgType.BYE:
                    self.close("BYE reçu")
        except (P.ConnectionClosed, P.ProtocolError, OSError, AudioError) as exc:
            self.close(str(exc))


class AudioServer:
    def __init__(self, settings: Settings) -> None:
        self._s = settings
        self._lock = threading.Lock()
        self._session: _Session | None = None
        self._mic: "queue.Queue[bytes]" = queue.Queue(maxsize=500)  # ~10 s de voix
        self._connected = threading.Event()
        self._stop = threading.Event()
        self._listener: socket.socket | None = None
        self._utt = 0

    # ---- cycle de vie ---------------------------------------------------
    def start(self) -> None:
        ls = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            ls.bind((self._s.audio_host, self._s.audio_port))
        except OSError as exc:
            raise AudioError(f"Port {self._s.audio_port} indisponible : {exc}") from exc
        ls.listen(1)
        ls.settimeout(1.0)
        self._listener = ls
        threading.Thread(target=self._accept_loop, name="audio-accept", daemon=True).start()
        net.info("Serveur audio en écoute sur %s:%d", self._s.audio_host, self._s.audio_port)

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            session = self._session
        if session:
            try:
                session.send(P.MsgType.BYE)
            except AudioError:
                pass
            session.close("arrêt du serveur")
        if self._listener:
            self._listener.close()

    @property
    def connected(self) -> bool:
        return self._connected.is_set()

    def wait_connected(self, timeout: float) -> bool:
        return self._connected.wait(timeout)

    # ---- acceptation ----------------------------------------------------
    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, addr = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                self._handshake(conn, addr)
            except Exception as exc:  # noqa: BLE001
                net.warning("Connexion %s rejetée : %s", addr[0], exc)
                conn.close()

    def _handshake(self, conn: socket.socket, addr) -> None:
        P.tune_socket(conn)
        conn.settimeout(1.0)
        reader = P.FrameReader(conn)
        deadline = time.monotonic() + 5.0
        while True:
            try:
                msg_type, payload = reader.read()
                break
            except socket.timeout:
                if time.monotonic() > deadline:
                    raise TimeoutError("HELLO non reçu")
        if msg_type != P.MsgType.HELLO:
            raise P.ProtocolError(f"HELLO attendu, reçu {msg_type.name}")
        hello = P.unpack_json(payload)
        if hello.get("version") != P.VERSION:
            raise P.ProtocolError(f"version {hello.get('version')} != {P.VERSION}")
        if hello.get("mic_rate") != self._s.audio_sample_rate or hello.get("mic_channels") != 1:
            raise P.ProtocolError(f"format micro inattendu : {hello}")
        P.send_all(conn, P.pack_json(P.MsgType.HELLO_ACK, {"version": P.VERSION}))
        session = _Session(self, conn, reader, addr, hello)
        with self._lock:
            old, self._session = self._session, session
        if old:
            old.close("remplacée par une nouvelle connexion")
        self.clear_mic()
        self._connected.set()
        session.start()
        net.info("Robot '%s' connecté depuis %s.", hello.get("robot_id"), addr[0])

    def _on_session_end(self, session: _Session) -> None:
        with self._lock:
            if self._session is session:
                self._session = None
                self._connected.clear()
        net.warning("Robot déconnecté.") if not self._stop.is_set() else None

    # ---- micro ----------------------------------------------------------
    def _push_mic(self, frame: bytes) -> None:
        try:
            self._mic.put_nowait(frame)
        except queue.Full:
            try:
                self._mic.get_nowait()
            except queue.Empty:
                pass
            self._mic.put_nowait(frame)

    def read_mic(self, timeout: float) -> bytes | None:
        try:
            return self._mic.get(timeout=timeout)
        except queue.Empty:
            return None

    def clear_mic(self) -> None:
        while True:
            try:
                self._mic.get_nowait()
            except queue.Empty:
                return

    # ---- haut-parleur ---------------------------------------------------
    def _current(self) -> _Session:
        with self._lock:
            session = self._session
        if session is None or session.closed.is_set():
            raise AudioNotConnectedError("aucun robot connecté")
        return session

    def play(self, chunks: Iterable[tuple[int, bytes]]) -> None:
        """Envoie (rate, pcm_s16le_mono) au robot et attend la fin réelle de la lecture."""
        session = self._current()
        self._utt += 1
        uid = self._utt
        done = session.expect_done(uid)
        rate = 0
        total = 0
        try:
            for chunk_rate, pcm in chunks:
                if rate == 0:
                    rate = chunk_rate
                    session.send(P.MsgType.SPK_BEGIN, P.pack_json(
                        P.MsgType.SPK_BEGIN, {"utterance_id": uid, "rate": rate, "channels": 1})[5:])
                elif chunk_rate != rate:
                    raise AudioError("fréquence audio variable dans une même phrase")
                for i in range(0, len(pcm), _CHUNK):
                    session.send(P.MsgType.SPK_DATA, pcm[i:i + _CHUNK])
                total += len(pcm)
            if rate == 0:
                return
            session.send(P.MsgType.SPK_END, P.pack_json(P.MsgType.SPK_END, {"utterance_id": uid})[5:])
        except BaseException:
            try:
                session.send(P.MsgType.SPK_CANCEL)
            except AudioError:
                pass
            raise
        log.info("Sending response to robot (%.1f s d'audio)", total / (2 * rate))
        if not done.wait(total / (2 * rate) + 10.0):
            try:
                session.send(P.MsgType.SPK_CANCEL)
            except AudioError:
                pass
            raise AudioTimeoutError("fin de lecture non confirmée par le robot")
        if session.closed.is_set():
            raise AudioConnectionLostError("connexion perdue pendant la lecture") 

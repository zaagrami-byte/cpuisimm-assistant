"""Protocole audio ISIMM v1 (robot <-> PC). Stdlib uniquement.

Trame : type (1 octet) | longueur (4 octets, big-endian) | payload.
Fichier partagé : ai/audio/protocol.py en est un lien symbolique.
"""
from __future__ import annotations

import json
import socket
import struct
import time
from enum import IntEnum

VERSION = 1
MAX_PAYLOAD = 1 << 20
_HEADER = struct.Struct("!BI")


class MsgType(IntEnum):
    HELLO = 1
    HELLO_ACK = 2
    MIC = 3
    SPK_BEGIN = 4
    SPK_DATA = 5
    SPK_END = 6
    SPK_CANCEL = 7
    SPK_DONE = 8
    PING = 9
    PONG = 10
    BYE = 11


class ProtocolError(Exception):
    """Trame invalide ou version incompatible."""


class ConnectionClosed(Exception):
    """Le pair a fermé la connexion."""


def pack(msg_type: MsgType, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError("payload trop grand")
    return _HEADER.pack(int(msg_type), len(payload)) + payload


def pack_json(msg_type: MsgType, obj: dict) -> bytes:
    return pack(msg_type, json.dumps(obj, separators=(",", ":")).encode("utf-8"))


def unpack_json(payload: bytes) -> dict:
    try:
        obj = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as exc:
        raise ProtocolError(f"JSON invalide : {exc}") from exc
    if not isinstance(obj, dict):
        raise ProtocolError("JSON attendu : objet")
    return obj


def tune_socket(sock: socket.socket) -> None:
    """TCP_NODELAY + keepalive agressif (Linux)."""
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    for name, value in (("TCP_KEEPIDLE", 10), ("TCP_KEEPINTVL", 3), ("TCP_KEEPCNT", 3)):
        if hasattr(socket, name):
            sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, name), value)


def send_all(sock: socket.socket, data: bytes, timeout: float = 10.0) -> None:
    """Envoi complet, sûr avec un socket à timeout (pas de trame tronquée)."""
    view = memoryview(data)
    deadline = time.monotonic() + timeout
    while view:
        try:
            sent = sock.send(view)
        except socket.timeout:
            if time.monotonic() > deadline:
                raise
            continue
        if sent == 0:
            raise ConnectionClosed("envoi impossible")
        view = view[sent:]


class FrameReader:
    """Lecture de trames ; un socket.timeout ne perd jamais d'octets déjà reçus."""

    def __init__(self, sock: socket.socket) -> None:
        self._sock = sock
        self._buf = bytearray()

    def read(self) -> tuple[MsgType, bytes]:
        while True:
            frame = self._try_parse()
            if frame is not None:
                return frame
            chunk = self._sock.recv(65536)
            if not chunk:
                raise ConnectionClosed("connexion fermée par le pair")
            self._buf += chunk

    def _try_parse(self) -> tuple[MsgType, bytes] | None:
        if len(self._buf) < _HEADER.size:
            return None
        raw_type, length = _HEADER.unpack_from(self._buf)
        if length > MAX_PAYLOAD:
            raise ProtocolError(f"trame trop grande ({length})")
        end = _HEADER.size + length
        if len(self._buf) < end:
            return None
        payload = bytes(self._buf[_HEADER.size:end])
        del self._buf[:end]
        try:
            return MsgType(raw_type), payload
        except ValueError as exc:
            raise ProtocolError(f"type inconnu {raw_type}") from exc 

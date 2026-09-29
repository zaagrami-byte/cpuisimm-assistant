import socket
import unittest

from assistant.voice import WakePolicy
from audio import protocol as P
from audio.vad import UtteranceSegmenter

FRAME = 640  # 20 ms @ 16 kHz s16


class TestProtocol(unittest.TestCase):
    def test_roundtrip_and_partial_reads(self):
        a, b = socket.socketpair()
        b.settimeout(0.2)
        data = P.pack(P.MsgType.MIC, b"x" * FRAME) + P.pack_json(P.MsgType.HELLO, {"version": 1})
        a.sendall(data[:10])
        reader = P.FrameReader(b)
        with self.assertRaises(socket.timeout):
            reader.read()                       # trame incomplète : aucun octet perdu
        a.sendall(data[10:])
        self.assertEqual(reader.read(), (P.MsgType.MIC, b"x" * FRAME))
        t, payload = reader.read()
        self.assertEqual((t, P.unpack_json(payload)), (P.MsgType.HELLO, {"version": 1}))
        a.close()
        with self.assertRaises(P.ConnectionClosed):
            reader.read()

    def test_oversized_frame_rejected(self):
        a, b = socket.socketpair()
        a.sendall(b"\x03" + (P.MAX_PAYLOAD + 1).to_bytes(4, "big"))
        with self.assertRaises(P.ProtocolError):
            P.FrameReader(b).read()


def segmenter(speech_flags):
    it = iter(speech_flags)
    return UtteranceSegmenter(lambda f: next(it), 16000, 20, 300, 0.7, 1000, 0.85, 15, 300)


class TestSegmenter(unittest.TestCase):
    def test_speech_then_silence_yields_utterance(self):
        flags = [True] * 40 + [False] * 60
        seg, out = segmenter(flags), None
        for _ in flags:
            out = seg.feed(b"\x01\x00" * (FRAME // 2)) or out
        self.assertIsNotNone(out)
        self.assertGreaterEqual(len(out), 40 * FRAME)

    def test_silence_only_yields_nothing(self):
        seg = segmenter([False] * 200)
        self.assertTrue(all(seg.feed(b"\x00" * FRAME) is None for _ in range(200)))

    def test_short_blip_dropped(self):
        flags = [True] * 15 + [False] * 60
        seg, out = segmenter(flags), None
        for _ in flags:
            out = seg.feed(b"\x01\x00" * (FRAME // 2)) or out
        self.assertIsNone(out)

    def test_rechunks_odd_sized_input(self):
        seg = segmenter([False] * 5)
        self.assertIsNone(seg.feed(b"\x00" * (FRAME + 100)))


class TestWake(unittest.TestCase):
    def test_disabled_passes_everything(self):
        self.assertEqual(WakePolicy(False, ("bonjour robot",), 30).filter("Va à l'accueil"), "Va à l'accueil")

    def test_requires_phrase_then_keeps_session(self):
        w = WakePolicy(True, ("bonjour robot",), 30)
        self.assertIsNone(w.filter("Va à l'accueil"))
        self.assertEqual(w.filter("Bonjour Robot, va à l'accueil"), "va à l'accueil")
        self.assertEqual(w.filter("Et maintenant au labo"), "Et maintenant au labo")

    def test_wake_phrase_alone_becomes_greeting(self):
        self.assertEqual(WakePolicy(True, ("hey isimm",), 30).filter("Hey ISIMM !"), "Bonjour")


if __name__ == "__main__":
    unittest.main() 

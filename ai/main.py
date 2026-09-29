"""Point d'entrée de l'assistant IA (PC).

  python main.py --text            # développement sans micro
  python main.py --text --no-ros   # sans robot ni ROS (test Qwen seul)
  python main.py --voice           # micro/haut-parleur du robot via le réseau
"""
from __future__ import annotations

import argparse
import dataclasses
import signal
import sys
import threading

from assistant.assistant import Assistant
from assistant.conversation import Conversation
from assistant.llm_client import LLMClient, LLMError
from assistant.tool_registry import ToolRegistry
from logsetup import get_logger, setup_logging
from ros2_interface.interface import OfflineRobot, RobotInterface
from settings import Settings, load_settings
from tools import build_registry
from tools.context import ToolContext
from tools.locations import LocationBook
from assistant.knowledge import KnowledgeBase

log = get_logger("AI")


def connect_robot(settings: Settings) -> RobotInterface:
    try:
        from ros2_interface.robot_bridge import RobotBridge
    except ImportError as exc:
        raise SystemExit(
            f"ROS 2 indisponible ({exc}). Faire : source /opt/ros/jazzy/setup.bash, et créer le venv "
            "avec --system-site-packages. Ou utiliser --no-ros.")
    return RobotBridge(settings)


def build_assistant(settings: Settings, robot: RobotInterface) -> tuple[Assistant, LLMClient, LocationBook]:
    locations = LocationBook.load(settings.locations_file, settings.map_frame)
    knowledge = KnowledgeBase.load(settings.knowledge_file)
    ctx = ToolContext(settings=settings, robot=robot, locations=locations)
    registry: ToolRegistry = build_registry(ctx)
    llm = LLMClient(settings)
    assistant = Assistant(settings, llm, registry, Conversation(settings.max_history_turns),
                          knowledge, locations)
    robot.add_nav_listener(assistant.on_nav_event)
    return assistant, llm, locations


def run_text(assistant: Assistant) -> None:
    assistant.set_announcer(lambda text: print(f"\n[AI] Annonce : {text}\nVous > ", end="", flush=True))
    print("Mode texte. Tapez votre message (/quit pour sortir).")
    while True:
        try:
            line = input("Vous > ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if line in ("/quit", "/exit"):
            return
        if line:
            print(f"Robot > {assistant.handle(line)}")


def run_voice(settings: Settings, assistant: Assistant, stop: threading.Event) -> None:
    import webrtcvad

    from assistant.voice import VoiceLoop, WakePolicy
    from audio.server import AudioServer
    from audio.vad import UtteranceSegmenter
    from speech.stt import WhisperSTT
    from speech.tts import PiperTTS

    stt = WhisperSTT(settings)
    stt.load()
    tts = PiperTTS(settings)
    tts.load()
    vad = webrtcvad.Vad(settings.vad_mode)
    segmenter = UtteranceSegmenter(
        is_speech=lambda frame: vad.is_speech(frame, settings.audio_sample_rate),
        sample_rate=settings.audio_sample_rate, frame_ms=settings.vad_frame_ms,
        start_window_ms=settings.vad_start_window_ms, start_ratio=settings.vad_start_ratio,
        end_silence_ms=settings.vad_end_silence_ms, end_ratio=settings.vad_end_ratio,
        max_utterance_s=settings.vad_max_utterance_s, min_utterance_ms=settings.vad_min_utterance_ms)
    server = AudioServer(settings)
    server.start()
    wake = WakePolicy(settings.wake_enabled, settings.wake_phrases, settings.wake_session_s)
    loop = VoiceLoop(server, stt, tts, assistant, segmenter, wake)
    assistant.set_announcer(loop.announce)
    log.info("Mode vocal prêt. En attente du robot sur le port %d.", settings.audio_port)
    try:
        loop.run(stop)
    finally:
        server.stop()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Assistant IA ISIMM (PC)")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--text", action="store_true")
    mode.add_argument("--voice", action="store_true")
    ap.add_argument("--config", default=None)
    ap.add_argument("--no-ros", action="store_true", help="aucun client ROS 2 (test IA seule)")
    dry = ap.add_mutually_exclusive_group()
    dry.add_argument("--dry-run", action="store_true", help="force AI_DRY_RUN=true")
    dry.add_argument("--live", action="store_true", help="force AI_DRY_RUN=false (robot réel)")
    args = ap.parse_args(argv)

    settings = load_settings(args.config)
    if args.dry_run:
        settings = dataclasses.replace(settings, dry_run=True)
    if args.live:
        settings = dataclasses.replace(settings, dry_run=False)
    setup_logging(settings.log_level, settings.log_dir)
    log.info("Démarrage (%s) — AI_DRY_RUN=%s, modèle=%s",
             "texte" if args.text else "vocal", settings.dry_run, settings.ollama_model)
    if settings.dry_run:
        log.warning("DRY RUN actif : navigate_to ne déplacera PAS le robot.")
    else:
        log.warning("MODE RÉEL : navigate_to enverra de vrais buts à Nav2.")

    robot: RobotInterface = OfflineRobot() if args.no_ros else connect_robot(settings)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    try:
        assistant, llm, _ = build_assistant(settings, robot)
        try:
            llm.check()
        except LLMError as exc:
            log.error("%s", exc)
            return 3
        if args.text:
            run_text(assistant)
        else:
            run_voice(settings, assistant, stop)
    finally:
        robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(main()) 

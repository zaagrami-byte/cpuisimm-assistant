"""Configuration centralisée : .env > assistant.yaml > valeurs par défaut."""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import yaml

AI_ROOT = Path(__file__).resolve().parent


def _load_dotenv(path: Path) -> None:
    if not path.is_file():
        return
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


def _bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on", "oui")


def _path(value) -> Path:
    p = Path(str(value)).expanduser()
    return p if p.is_absolute() else AI_ROOT / p


@dataclass(frozen=True)
class Settings:
    language: str
    dry_run: bool
    log_level: str
    log_dir: Path
    ollama_host: str
    ollama_model: str
    llm_timeout_s: float
    llm_temperature: float
    llm_num_ctx: int
    llm_keep_alive: str
    max_history_turns: int
    max_tool_rounds: int
    whisper_model: str
    whisper_device: str
    whisper_compute_type: str
    whisper_beam_size: int
    stt_timeout_s: float
    stt_model_dir: Path
    stt_ignore_phrases: tuple[str, ...]
    piper_model: Path
    vad_mode: int
    vad_frame_ms: int
    vad_start_window_ms: int
    vad_start_ratio: float
    vad_end_silence_ms: int
    vad_end_ratio: float
    vad_max_utterance_s: float
    vad_min_utterance_ms: int
    audio_host: str
    audio_port: int
    audio_sample_rate: int
    audio_peer_timeout_s: float
    audio_ping_interval_s: float
    ros_domain_id: int
    nav_action: str
    map_frame: str
    base_frame: str
    status_timeout_s: float
    stop_mode: str
    wake_enabled: bool
    wake_phrases: tuple[str, ...]
    wake_session_s: float
    knowledge_file: Path
    locations_file: Path
    pc_ip: str
    robot_ip: str
    robot_host: str


def load_settings(config_path: str | Path | None = None) -> Settings:
    _load_dotenv(AI_ROOT / ".env")
    cfg_file = _path(config_path or os.environ.get("AI_CONFIG", "config/assistant.yaml"))
    if not cfg_file.is_file():
        raise FileNotFoundError(f"Fichier de configuration introuvable : {cfg_file}")
    data = yaml.safe_load(cfg_file.read_text(encoding="utf-8")) or {}

    def y(section: str, key: str, default):
        return (data.get(section) or {}).get(key, default)

    def env(name: str, default, cast=str):
        raw = os.environ.get(name)
        return cast(raw) if raw not in (None, "") else default

    s = Settings(
        language=env("AI_LANGUAGE", data.get("language", "fr")),
        dry_run=env("AI_DRY_RUN", _bool(data.get("dry_run", True)), _bool),
        log_level=env("AI_LOG_LEVEL", data.get("log_level", "INFO")).upper(),
        log_dir=_path(data.get("log_dir", "logs")),
        ollama_host=env("OLLAMA_HOST", y("llm", "host", "http://localhost:11434")),
        ollama_model=env("OLLAMA_MODEL", y("llm", "model", "qwen2.5:3b")),
        llm_timeout_s=float(y("llm", "timeout_s", 40)),
        llm_temperature=float(y("llm", "temperature", 0.2)),
        llm_num_ctx=int(y("llm", "num_ctx", 4096)),
        llm_keep_alive=str(y("llm", "keep_alive", "30m")),
        max_history_turns=int(y("llm", "max_history_turns", 6)),
        max_tool_rounds=int(y("llm", "max_tool_rounds", 4)),
        whisper_model=env("WHISPER_MODEL", y("stt", "model", "small")),
        whisper_device=str(y("stt", "device", "cpu")),
        whisper_compute_type=str(y("stt", "compute_type", "int8")),
        whisper_beam_size=int(y("stt", "beam_size", 1)),
        stt_timeout_s=float(y("stt", "timeout_s", 30)),
        stt_model_dir=_path(y("stt", "model_dir", "models/whisper")),
        stt_ignore_phrases=tuple(y("stt", "ignore_phrases", [])),
        piper_model=_path(env("PIPER_MODEL", y("tts", "model", "models/fr_FR-siwis-medium.onnx"))),
        vad_mode=int(y("vad", "mode", 3)),
        vad_frame_ms=int(y("vad", "frame_ms", 20)),
        vad_start_window_ms=int(y("vad", "start_window_ms", 300)),
        vad_start_ratio=float(y("vad", "start_ratio", 0.7)),
        vad_end_silence_ms=int(y("vad", "end_silence_ms", 1000)),
        vad_end_ratio=float(y("vad", "end_ratio", 0.85)),
        vad_max_utterance_s=float(y("vad", "max_utterance_s", 15)),
        vad_min_utterance_ms=int(y("vad", "min_utterance_ms", 300)),
        audio_host=env("AUDIO_HOST", y("audio", "bind_host", "0.0.0.0")),
        audio_port=env("AUDIO_PORT", int(y("audio", "port", 5005)), int),
        audio_sample_rate=int(y("audio", "sample_rate", 16000)),
        audio_peer_timeout_s=float(y("audio", "peer_timeout_s", 6)),
        audio_ping_interval_s=float(y("audio", "ping_interval_s", 2)),
        ros_domain_id=env("ROS_DOMAIN_ID", int(y("ros", "domain_id", 0)), int),
        nav_action=str(y("ros", "nav_action", "navigate_to_pose")),
        map_frame=str(y("ros", "map_frame", "map")),
        base_frame=str(y("ros", "base_frame", "base_link")),
        status_timeout_s=float(y("ros", "status_timeout_s", 3.0)),
        stop_mode=str(y("robot", "stop_mode", "estop")),
        wake_enabled=_bool(y("wake", "enabled", False)),
        wake_phrases=tuple(y("wake", "phrases", ["bonjour robot", "hey isimm"])),
        wake_session_s=float(y("wake", "session_s", 30)),
        knowledge_file=_path(data.get("knowledge_file", "knowledge/isimm_knowledge.yaml")),
        locations_file=_path(data.get("locations_file", "knowledge/locations.yaml")),
        pc_ip=env("PC_IP", ""),
        robot_ip=env("ROBOT_IP", ""),
        robot_host=env("ROBOT_HOST", ""),
    )
    if s.stop_mode not in ("estop", "cancel"):
        raise ValueError("robot.stop_mode doit valoir 'estop' ou 'cancel'")
    if s.vad_frame_ms not in (10, 20, 30) or not 0 <= s.vad_mode <= 3:
        raise ValueError("vad.frame_ms ∈ {10,20,30} et vad.mode ∈ [0,3]")
    return s 

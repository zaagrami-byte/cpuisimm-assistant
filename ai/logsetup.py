"""Logs taggés : [AI] [LLM] [STT] [TTS] [AUDIO] [TOOL] [ROS2] [NAV2] [NETWORK] [SAFETY]."""
from __future__ import annotations

import logging
import logging.handlers
from pathlib import Path

_FMT = "%(asctime)s %(levelname)-7s [%(name)s] %(message)s"
_configured = False


def setup_logging(level: str, log_dir: Path) -> None:
    global _configured
    if _configured:
        return
    _configured = True
    log_dir.mkdir(parents=True, exist_ok=True)
    root = logging.getLogger()
    root.setLevel(getattr(logging, level, logging.INFO))
    formatter = logging.Formatter(_FMT, "%H:%M:%S")
    console = logging.StreamHandler()
    console.setFormatter(formatter)
    root.addHandler(console)
    file_handler = logging.handlers.RotatingFileHandler(
        log_dir / "ai.log", maxBytes=2_000_000, backupCount=3, encoding="utf-8")
    file_handler.setFormatter(logging.Formatter(_FMT, "%Y-%m-%d %H:%M:%S"))
    root.addHandler(file_handler)
    for noisy in ("httpx", "httpcore", "faster_whisper", "urllib3"):
        logging.getLogger(noisy).setLevel(logging.WARNING)


def get_logger(tag: str) -> logging.Logger:
    return logging.getLogger(tag)

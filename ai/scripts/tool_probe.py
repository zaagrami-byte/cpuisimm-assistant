"""Exécute un outil SANS LLM (tests 8 à 14).

  python -m scripts.tool_probe --list
  python -m scripts.tool_probe get_robot_status
  python -m scripts.tool_probe navigate_to '{"location":"accueil"}' [--live]
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import sys

from logsetup import setup_logging
from main import connect_robot
from ros2_interface.interface import OfflineRobot
from settings import load_settings
from tools import build_registry
from tools.context import ToolContext
from tools.locations import LocationBook


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("tool", nargs="?")
    ap.add_argument("args", nargs="?", default="{}")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--no-ros", action="store_true")
    ap.add_argument("--live", action="store_true", help="AI_DRY_RUN=false")
    a = ap.parse_args()
    s = load_settings()
    if a.live:
        s = dataclasses.replace(s, dry_run=False)
    setup_logging(s.log_level, s.log_dir)
    robot = OfflineRobot() if a.no_ros else connect_robot(s)
    try:
        registry = build_registry(ToolContext(s, robot, LocationBook.load(s.locations_file, s.map_frame)))
        if a.list or not a.tool:
            print("\n".join(registry.names()))
            return 0
        import time
        time.sleep(2.0)  # laisse la découverte DDS et les premiers messages arriver
        print(json.dumps(registry.execute(a.tool, json.loads(a.args)), ensure_ascii=False, indent=2))
    finally:
        robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(main()) 

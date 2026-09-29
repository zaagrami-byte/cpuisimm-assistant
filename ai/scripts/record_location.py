"""Relève la pose actuelle du robot et affiche le bloc à coller dans knowledge/locations.yaml.

  python -m scripts.record_location administration
"""
from __future__ import annotations

import math
import sys
import time

from main import connect_robot
from ros2_interface.interface import PoseUnavailableError
from settings import load_settings


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    name = sys.argv[1]
    robot = connect_robot(load_settings())
    try:
        pose = None
        for _ in range(20):
            try:
                pose = robot.get_pose()
                break
            except PoseUnavailableError:
                time.sleep(0.5)
        if pose is None:
            print("TF map->base_link indisponible : la localisation (navigation.launch.py) tourne-t-elle ?")
            return 1
        print(f"  {name}:\n    x: {pose.x:.2f}\n    y: {pose.y:.2f}\n    yaw_deg: {math.degrees(pose.yaw):.1f}")
    finally:
        robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(main()) 

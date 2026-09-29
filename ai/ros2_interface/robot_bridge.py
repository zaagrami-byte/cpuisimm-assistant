"""Pont ROS 2 (PC) vers le Raspberry Pi. LECTURE + action Nav2 + /emergency_stop.
Ne publie JAMAIS /cmd_vel."""
from __future__ import annotations

import collections
import math
import os
import threading
import time
from typing import Callable

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from logsetup import get_logger
from ros2_interface.interface import (NavSnapshot, Pose2D, PoseUnavailableError, RobotUnavailableError)
from ros2_interface.nav2_client import Nav2Client
from settings import Settings

log = get_logger("ROS2")
_LEVELS = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


def _level(value) -> int:
    return value[0] if isinstance(value, (bytes, bytearray)) else int(value)


class RobotBridge:
    def __init__(self, settings: Settings) -> None:
        self._s = settings
        os.environ["ROS_DOMAIN_ID"] = str(settings.ros_domain_id)
        if not rclpy.ok():
            rclpy.init()
        self._node = Node("ai_robot_bridge")
        self._lock = threading.Lock()
        self._diag: list[dict] = []
        self._diag_t: float | None = None
        self._safety: str | None = None
        self._motor: str | None = None
        self._cmd = collections.deque(maxlen=200)    # (t, |v| commandé)
        self._odom = collections.deque(maxlen=200)   # (t, |v| mesuré)

        node = self._node
        node.create_subscription(DiagnosticArray, '/robot_status', self._on_diag, 10)
        node.create_subscription(String, '/safety_state', self._on_safety, 10)
        node.create_subscription(String, '/motor_status', self._on_motor, 10)
        node.create_subscription(Twist, '/safe_cmd_vel', self._on_cmd, 10)
        node.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self._estop_pub = node.create_publisher(Bool, '/emergency_stop', 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self._nav = Nav2Client(node, settings)

        self._executor = MultiThreadedExecutor(num_threads=3)
        self._executor.add_node(node)
        threading.Thread(target=self._executor.spin, name="ros2-executor", daemon=True).start()
        log.info("Pont ROS 2 démarré (ROS_DOMAIN_ID=%d, nœud /ai_robot_bridge)", settings.ros_domain_id)

    # ---- callbacks -------------------------------------------------------
    def _on_diag(self, msg: DiagnosticArray) -> None:
        items = [{"name": st.name.removeprefix("isimm/"), "level": _level(st.level), "message": st.message}
                 for st in msg.status]
        with self._lock:
            self._diag, self._diag_t = items, time.monotonic()

    def _on_safety(self, msg: String) -> None:
        with self._lock:
            self._safety = msg.data

    def _on_motor(self, msg: String) -> None:
        with self._lock:
            self._motor = msg.data

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd.append((time.monotonic(), abs(msg.linear.x)))

    def _on_odom(self, msg: Odometry) -> None:
        self._odom.append((time.monotonic(), abs(msg.twist.twist.linear.x)))

    # ---- lecture ---------------------------------------------------------
    def get_status(self) -> dict:
        with self._lock:
            diag, t, safety, motor = list(self._diag), self._diag_t, self._safety, self._motor
        if t is None or time.monotonic() - t > self._s.status_timeout_s:
            raise RobotUnavailableError("aucun /robot_status récent (Pi injoignable ou DDS coupé)")
        overall = next((d for d in diag if d["name"] == "overall"), None)
        return {
            "overall_level": overall["level"] if overall else 3,
            "overall_message": overall["message"] if overall else "inconnu",
            "components": [d for d in diag if d["name"] != "overall"],
            "safety_state": safety,
            "motor_status": motor,
        }

    def get_pose(self) -> Pose2D:
        try:
            tr = self._tf_buffer.lookup_transform(self._s.map_frame, self._s.base_frame, Time())
        except TransformException as exc:
            raise PoseUnavailableError(str(exc)) from exc
        q = tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return Pose2D(tr.transform.translation.x, tr.transform.translation.y, yaw)

    @staticmethod
    def _avg(samples, window: float) -> tuple[float, int]:
        now = time.monotonic()
        values = [v for t, v in list(samples) if now - t <= window]
        return (sum(values) / len(values), len(values)) if values else (0.0, 0)

    def is_moving(self) -> bool | None:
        avg, n = self._avg(self._odom, 2.0)
        return None if n < 2 else avg > 0.03

    def get_observations(self) -> list[str]:
        """Observations seulement : l'IA n'agit jamais sur la calibration."""
        obs: list[str] = []
        cmd, n_cmd = self._avg(self._cmd, 3.0)
        meas, n_odom = self._avg(self._odom, 3.0)
        if n_cmd >= 20 and n_odom >= 3 and cmd >= 0.08 and meas < 0.5 * cmd:
            obs.append(f"La vitesse mesurée ({meas:.2f} m/s) semble inférieure à la vitesse demandée "
                       f"({cmd:.2f} m/s). Une calibration moteur/odométrie peut être nécessaire.")
        with self._lock:
            safety, motor = self._safety, self._motor
        if safety not in (None, "OK", "NO_CMD_YET", "CMD_TIMEOUT"):
            obs.append(f"Le nœud de sécurité bloque actuellement le mouvement ({safety}).")
        if motor not in (None, "OK", "STARTING"):
            obs.append(f"État de la liaison moteur : {motor}.")
        return obs

    # ---- navigation / arrêt ---------------------------------------------
    def add_nav_listener(self, callback: Callable[[NavSnapshot], None]) -> None:
        self._nav.add_listener(callback)

    def nav_snapshot(self) -> NavSnapshot:
        return self._nav.snapshot()

    def navigate(self, name: str, x: float, y: float, yaw: float) -> NavSnapshot:
        return self._nav.navigate(name, x, y, yaw)

    def cancel_navigation(self) -> bool:
        return self._nav.cancel()

    def emergency_stop(self) -> bool:
        """Publie /emergency_stop=True (lu par safety_node et motor_controller). True si confirmé."""
        if self._estop_pub.get_subscription_count() == 0:
            raise RobotUnavailableError("aucun abonné à /emergency_stop (Pi injoignable)")
        log.warning("Publication de /emergency_stop=True")
        msg = Bool(data=True)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self._estop_pub.publish(msg)
            time.sleep(0.1)
            with self._lock:
                if self._safety == "EMERGENCY_STOP":
                    return True
        return False

    def close(self) -> None:
        try:
            self._executor.shutdown()
            self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass 

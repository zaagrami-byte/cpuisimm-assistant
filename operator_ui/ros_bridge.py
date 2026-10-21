"""Pont ROS 2 (PC) : abonnements, publications, opérations de sécurité.

Threads : rclpy tourne dans son propre thread (executor). Ce module n'accède JAMAIS aux widgets :
il émet des signaux Qt (Snapshot à 10 Hz, résultats d'opérations).
Publie : /cmd_vel_teleop (JAMAIS /cmd_vel), /manual_session, /emergency_stop.
Appelle : /motor_controller_node/clear_estop, /navigate_to_pose/_action/cancel_goal.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Callable, Optional

import numpy as np
from PySide6.QtCore import QObject, Signal

from .model import Scan, Snapshot

try:
    import rclpy
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from action_msgs.srv import CancelGoal
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PolygonStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    ROS_IMPORT_ERROR = ""
except ImportError as _exc:            # ROS 2 non sourcé : l'interface reste utilisable
    rclpy = None
    ROS_IMPORT_ERROR = str(_exc)

# Topics / services (cohérents avec ros2_ws : voir tests/test_consistency.py)
T_TELEOP, T_SESSION, T_ESTOP = "/cmd_vel_teleop", "/manual_session", "/emergency_stop"
T_STATUS, T_SAFETY, T_MOTOR, T_MODE = "/robot_status", "/safety_state", "/motor_status", "/cmd_vel_mode"
T_SAFE_CMD, T_ODOM, T_SCAN = "/safe_cmd_vel", "/odom", "/scan"
T_ZONE_STOP, T_ZONE_SLOW = "/polygon_stop", "/polygon_slowdown"
T_GOAL_STATUS = "/navigate_to_pose/_action/status"
S_CANCEL, S_CLEAR_ESTOP = "/navigate_to_pose/_action/cancel_goal", "/motor_controller_node/clear_estop"
BASE_FRAME = "base_link"
SAFE_AT_REST = ("OK", "NO_CMD_YET", "CMD_TIMEOUT")


def _level(value) -> int:
    return value[0] if isinstance(value, (bytes, bytearray)) else int(value)


class RosBridge(QObject):
    updated = Signal(object)            # Snapshot
    op_done = Signal(str, bool, str)    # opération, succès, message (français)

    def __init__(self, status_stale_s: float = 3.0) -> None:
        super().__init__()
        self._stale = status_stale_s
        self._lock = threading.Lock()
        self._v: dict = {}               # clé -> (valeur, t monotonic)
        self._session_on = False
        self._op_lock = threading.Lock()
        self._node = None
        self._executor = None
        self._error = ROS_IMPORT_ERROR

    # ---------------------------------------------------------------- cycle de vie
    def start(self) -> bool:
        if rclpy is None:
            self._publish_snapshot()
            return False
        try:
            if not rclpy.ok():
                rclpy.init()
            self._build_node()
        except Exception as exc:        # noqa: BLE001
            self._error = f"initialisation ROS 2 impossible : {exc}"
            self._node = None
            self._publish_snapshot()
            return False
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        threading.Thread(target=self._executor.spin, name="ros2-ui", daemon=True).start()
        return True

    def _build_node(self) -> None:
        n = self._node = Node("operator_ui")
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        n.create_subscription(DiagnosticArray, T_STATUS, self._on_diag, 10)
        n.create_subscription(String, T_SAFETY, lambda m: self._set("safety", m.data), 10)
        n.create_subscription(String, T_MOTOR, lambda m: self._set("motor", m.data), 10)
        n.create_subscription(String, T_MODE, lambda m: self._set("mode", m.data), 10)
        n.create_subscription(Twist, T_SAFE_CMD,
                              lambda m: self._set("cmd", (m.linear.x, m.angular.z)), 10)
        n.create_subscription(Odometry, T_ODOM, lambda m: self._set(
            "odom", (m.twist.twist.linear.x, m.twist.twist.angular.z)), 10)
        n.create_subscription(LaserScan, T_SCAN, self._on_scan, qos_profile_sensor_data)
        n.create_subscription(PolygonStamped, T_ZONE_STOP, lambda m: self._on_zone("zone_stop", m), be)
        n.create_subscription(PolygonStamped, T_ZONE_SLOW, lambda m: self._on_zone("zone_slow", m), be)
        n.create_subscription(GoalStatusArray, T_GOAL_STATUS, self._on_goals, latched)
        self._teleop_pub = n.create_publisher(Twist, T_TELEOP, 10)
        self._session_pub = n.create_publisher(Bool, T_SESSION, 10)
        self._estop_pub = n.create_publisher(Bool, T_ESTOP, 10)
        self._cancel_cli = n.create_client(CancelGoal, S_CANCEL)
        self._clear_cli = n.create_client(Trigger, S_CLEAR_ESTOP)
        n.create_timer(0.1, self._tick)

    def shutdown(self) -> None:
        """Arrêt propre : commande nulle, fin de session, annulation Nav2 (best effort)."""
        if self._node is None:
            return
        self._session_on = False
        try:
            for _ in range(3):
                self._teleop_pub.publish(Twist())
                self._session_pub.publish(Bool(data=False))
                time.sleep(0.03)
            self.cancel_all_goals(timeout=1.5)
        except Exception:               # noqa: BLE001
            pass
        try:
            self._executor.shutdown()
            self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:               # noqa: BLE001
            pass
        self._node = None

    # ---------------------------------------------------------------- callbacks (thread ROS)
    def _set(self, key, value) -> None:
        with self._lock:
            self._v[key] = (value, time.monotonic())

    def _on_diag(self, msg) -> None:
        comps = {st.name.removeprefix("isimm/"): (_level(st.level), st.message) for st in msg.status}
        self._set("diag", comps)

    def _on_scan(self, msg) -> None:
        self._set("scan", Scan(np.asarray(msg.ranges, dtype=np.float32), msg.angle_min,
                               msg.angle_increment, msg.range_min, msg.range_max,
                               msg.header.frame_id))

    def _on_zone(self, key, msg) -> None:
        if msg.header.frame_id != BASE_FRAME or not msg.polygon.points:
            return                       # on ne dessine que ce qui est dans le bon repère
        self._set(key, np.array([[p.x, p.y] for p in msg.polygon.points], dtype=np.float32))

    def _on_goals(self, msg) -> None:
        active = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)
        self._set("goals", sum(1 for s in msg.status_list if s.status in active))

    def _tick(self) -> None:
        if self._session_on:             # heartbeat de session manuelle (10 Hz)
            self._session_pub.publish(Bool(data=True))
        self._publish_snapshot()

    # ---------------------------------------------------------------- snapshot
    def _get(self, key, ttl: Optional[float] = None):
        with self._lock:
            item = self._v.get(key)
        if item is None:
            return None, None
        value, t = item
        age = time.monotonic() - t
        return (None, age) if ttl is not None and age > ttl else (value, age)

    def snapshot(self) -> Snapshot:
        diag, diag_age = self._get("diag")
        connected = diag is not None and diag_age <= self._stale
        cmd, _ = self._get("cmd", 1.0)
        odom, _ = self._get("odom", 1.0)
        scan, scan_age = self._get("scan")
        goals, _ = self._get("goals")
        return Snapshot(
            ros_available=self._node is not None, ros_error=self._error,
            robot_connected=connected, components=dict(diag) if connected else {},
            safety_state=self._get("safety", 3.0)[0], motor_status=self._get("motor", 3.0)[0],
            mux_mode=self._get("mode", 1.5)[0],
            cmd_v=cmd[0] if cmd else None, cmd_w=cmd[1] if cmd else None,
            meas_v=odom[0] if odom else None, meas_w=odom[1] if odom else None,
            scan=scan, scan_age=scan_age,
            zone_stop=self._get("zone_stop", 2.0)[0], zone_slow=self._get("zone_slow", 2.0)[0],
            nav_goals_active=goals, session_on=self._session_on)

    def _publish_snapshot(self) -> None:
        self.updated.emit(self.snapshot())

    # ---------------------------------------------------------------- commandes manuelles
    @property
    def session_on(self) -> bool:
        return self._session_on

    def publish_teleop(self, v: float, w: float) -> None:
        """Appelé à 20 Hz par l'interface. Rien n'est publié hors session confirmée."""
        if self._node is None or not self._session_on:
            return
        if not (math.isfinite(v) and math.isfinite(w)):
            v = w = 0.0
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self._teleop_pub.publish(m)

    def stop_motion(self) -> None:
        """Arrêt immédiat côté interface : zéro, fin de session (le mux gère le reste)."""
        self._session_on = False
        if self._node is not None:
            self._teleop_pub.publish(Twist())
            self._session_pub.publish(Bool(data=False))

    # ---------------------------------------------------------------- opérations asynchrones
    def _run(self, op: str, fn: Callable[[], tuple[bool, str]], exclusive: bool = True) -> None:
        def work():
            if exclusive and not self._op_lock.acquire(blocking=False):
                self.op_done.emit(op, False, "Une autre opération est en cours.")
                return
            try:
                ok, msg = fn()
            except Exception as exc:    # noqa: BLE001
                ok, msg = False, f"Erreur interne : {exc}"
            finally:
                if exclusive and self._op_lock.locked():
                    self._op_lock.release()
            self.op_done.emit(op, ok, msg)
        threading.Thread(target=work, name=f"op-{op}", daemon=True).start()

    @staticmethod
    def _wait_future(fut, timeout: float):
        done = threading.Event()
        fut.add_done_callback(lambda _: done.set())
        return fut.result() if done.wait(timeout) else None

    def cancel_all_goals(self, timeout: float = 3.0) -> tuple[bool, str]:
        """Annule TOUS les buts Nav2 puis vérifie qu'aucun n'est encore actif."""
        if self._node is None or not self._cancel_cli.wait_for_service(timeout_sec=2.0):
            return False, "service d'annulation Nav2 indisponible"
        resp = self._wait_future(self._cancel_cli.call_async(CancelGoal.Request()), timeout)
        if resp is None:
            return False, "annulation Nav2 sans réponse"
        if resp.return_code == CancelGoal.Response.ERROR_REJECTED:
            return False, "annulation Nav2 refusée"
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            goals, _ = self._get("goals")
            if not goals:                # 0 ou jamais reçu (aucun but n'a existé)
                return True, "aucun but Nav2 actif"
            time.sleep(0.1)
        return False, "un but Nav2 est encore actif après l'annulation"

    def request_manual(self, enable: bool) -> None:
        self._run("manual_on" if enable else "manual_off",
                  self._enable_manual if enable else self._disable_manual)

    def _enable_manual(self) -> tuple[bool, str]:
        self._session_on = False
        ok, msg = self.cancel_all_goals()
        if not ok:
            return False, f"Activation refusée : {msg}."
        self._teleop_pub.publish(Twist())
        self._session_on = True          # le heartbeat démarre
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if self._get("mode", 1.5)[0] == "MANUAL":
                return True, "Pilotage manuel activé (buts Nav2 annulés, mux en MANUAL)."
            time.sleep(0.05)
        self.stop_motion()
        return False, "Activation annulée : le multiplexeur n'est pas passé en MANUAL."

    def _disable_manual(self) -> tuple[bool, str]:
        self.stop_motion()
        ok, msg = self.cancel_all_goals()
        if ok:
            return True, "Pilotage manuel désactivé. Le robot attend une nouvelle commande automatique."
        return True, f"Pilotage désactivé, mais vérifiez Nav2 : {msg}."

    def trigger_estop(self) -> None:
        """Arrêt d'urgence : immédiat côté interface, répété jusqu'à confirmation du safety_node."""
        self.stop_motion()
        if self._node is None:
            self.op_done.emit("estop", False, "ROS 2 indisponible : AUCUN ordre envoyé. "
                              "Utilisez l'arrêt matériel.")
            return

        def work():
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                self._estop_pub.publish(Bool(data=True))
                if self._get("safety", 3.0)[0] == "EMERGENCY_STOP":
                    self.op_done.emit("estop", True, "Arrêt d'urgence confirmé par le safety_node.")
                    return
                time.sleep(0.1)
            self.op_done.emit("estop", False, "Ordre envoyé mais NON confirmé : vérifiez le robot / "
                              "utilisez l'arrêt matériel.")
        threading.Thread(target=work, name="estop", daemon=True).start()

    def request_rearm(self) -> None:
        self._run("rearm", self._rearm)

    def _rearm(self) -> tuple[bool, str]:
        if self._node is None:
            return False, "ROS 2 indisponible."
        self.stop_motion()
        self._estop_pub.publish(Bool(data=False))
        ok = False
        if self._clear_cli.wait_for_service(timeout_sec=2.0):
            resp = self._wait_future(self._clear_cli.call_async(Trigger.Request()), 2.0)
            ok = bool(resp and resp.success)
        if not ok:
            return False, "Réarmement : clear_estop du contrôleur moteur indisponible ou refusé."
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            self._estop_pub.publish(Bool(data=False))
            safety, motor = self._get("safety", 3.0)[0], self._get("motor", 3.0)[0]
            mode = self._get("mode", 1.5)[0]
            if safety in SAFE_AT_REST and motor == "OK" and mode != "ESTOP":
                return True, ("Réarmé : sécurité et contrôleur moteur confirment un état sûr. "
                              "Aucun mouvement n'est lancé ; réactivez le pilotage manuel si besoin.")
            time.sleep(0.2)
        safety, motor = self._get("safety", 3.0)[0], self._get("motor", 3.0)[0]
        hint = " Arrêt matériel (bouton physique) probablement encore enfoncé : non réarmable par ROS." \
            if motor == "ARDUINO_ESTOP" else ""
        return False, f"Réarmement NON confirmé (sécurité={safety}, moteur={motor}).{hint}"

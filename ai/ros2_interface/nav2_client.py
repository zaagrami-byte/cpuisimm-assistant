"""Client NavigateToPose : un seul but actif, annulation confirmée avant tout nouveau but."""
from __future__ import annotations

import math
import threading
import time
from typing import Callable

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from logsetup import get_logger
from ros2_interface.interface import NavigationError, Nav2UnavailableError, NavSnapshot, NavState
from settings import Settings

log = get_logger("NAV2")


class Nav2Client:
    def __init__(self, node, settings: Settings) -> None:
        self._map_frame = settings.map_frame
        self._client = ActionClient(node, NavigateToPose, settings.nav_action)
        self._cmd_lock = threading.RLock()   # sérialise navigate/cancel (jamais 2 navigations)
        self._lock = threading.Lock()        # protège l'état
        self._handle = None
        self._gen = 0
        self._silent_gen: int | None = None
        self._state = NavState.IDLE
        self._dest: str | None = None
        self._detail = ""
        self._dist: float | None = None
        self._listeners: list[Callable[[NavSnapshot], None]] = []

    def add_listener(self, cb: Callable[[NavSnapshot], None]) -> None:
        self._listeners.append(cb)

    def available(self) -> bool:
        return self._client.server_is_ready()

    def snapshot(self) -> NavSnapshot:
        with self._lock:
            state, dest, detail, dist = self._state, self._dest, self._detail, self._dist
        if state in (NavState.IDLE, NavState.NAVIGATING) and not self._client.server_is_ready():
            return NavSnapshot(NavState.UNKNOWN, dest, "Nav2 injoignable", dist)
        return NavSnapshot(state, dest, detail, dist)

    # ---- commandes -------------------------------------------------------
    def navigate(self, name: str, x: float, y: float, yaw: float, timeout: float = 5.0) -> NavSnapshot:
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise Nav2UnavailableError("serveur d'action indisponible")
        with self._cmd_lock:
            self._cancel_locked(silent=True)
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = self._map_frame   # stamp=0 : dernière TF (horloges PC/Pi indépendantes)
            goal.pose.pose.position.x = float(x)
            goal.pose.pose.position.y = float(y)
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
            log.info("Sending NavigateToPose -> %s (x=%.2f y=%.2f yaw=%.2f)", name, x, y, yaw)
            handle = self._wait(self._client.send_goal_async(goal, feedback_callback=self._on_feedback),
                                timeout)
            if handle is None:
                raise NavigationError("délai dépassé lors de l'envoi du but")
            if not handle.accepted:
                with self._lock:
                    self._state, self._dest, self._detail = NavState.FAILED, name, "but refusé par Nav2"
                raise NavigationError("Nav2 a refusé cette destination")
            with self._lock:
                self._gen += 1
                gen = self._gen
                self._handle, self._state, self._dest = handle, NavState.NAVIGATING, name
                self._detail, self._dist = "", None
            handle.get_result_async().add_done_callback(lambda fut, g=gen: self._on_result(g, fut))
            log.info("Goal accepted")
            return self.snapshot()

    def cancel(self) -> bool:
        """True si un but actif a été annulé, False s'il n'y en avait pas."""
        if not self._client.server_is_ready():
            raise Nav2UnavailableError("serveur d'action indisponible")
        with self._cmd_lock:
            return self._cancel_locked(silent=False)

    def _cancel_locked(self, silent: bool) -> bool:
        with self._lock:
            handle = self._handle
            if self._state != NavState.NAVIGATING or handle is None:
                return False
            self._silent_gen = self._gen if silent else None
        log.info("Cancelling current goal")
        if self._wait(handle.cancel_goal_async(), 3.0) is None:
            raise NavigationError("annulation non confirmée (délai dépassé)")
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            with self._lock:
                if self._state != NavState.NAVIGATING:
                    return True
            time.sleep(0.05)
        raise NavigationError("le but précédent n'est pas confirmé annulé")

    # ---- callbacks -------------------------------------------------------
    def _on_feedback(self, msg) -> None:
        with self._lock:
            self._dist = float(msg.feedback.distance_remaining)

    def _on_result(self, gen: int, future) -> None:
        try:
            wrapped = future.result()
            status, result = wrapped.status, wrapped.result
        except Exception as exc:  # noqa: BLE001
            status, result = None, None
            log.error("Résultat Nav2 illisible : %s", exc)
        if status == GoalStatus.STATUS_SUCCEEDED:
            state, detail = NavState.SUCCEEDED, ""
            log.info("Goal reached")
        elif status == GoalStatus.STATUS_CANCELED:
            state, detail = NavState.CANCELED, ""
            log.info("Goal canceled")
        else:
            state = NavState.FAILED
            code = getattr(result, "error_code", 0)
            msg = getattr(result, "error_msg", "")
            detail = f"code {code} {msg}".strip()
            log.warning("Goal failed (%s)", detail)
        with self._lock:
            if gen != self._gen:
                return
            self._state, self._detail, self._handle = state, detail, None
            notify = self._silent_gen != gen
            snap = NavSnapshot(state, self._dest, detail, self._dist)
        if notify:
            for cb in self._listeners:
                try:
                    cb(snap)
                except Exception:  # noqa: BLE001
                    log.exception("Listener de navigation en erreur")

    @staticmethod
    def _wait(future, timeout: float):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        return future.result() if done.wait(timeout) else None 

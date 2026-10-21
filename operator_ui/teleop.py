"""Logique pure du pilotage manuel : configuration, commande à partir de la manette, conditions
d'activation. Aucun import Qt/ROS/pygame (testable partout)."""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml

from .model import PadState, Snapshot

CONFIG_DIR = Path(__file__).resolve().parent / "config"
HARD_MAX_LINEAR = 0.30      # l'interface refuse une config au-delà (le robot écrête de toute façon)
HARD_MAX_ANGULAR = 1.0
OK_SAFETY = ("OK",)


@dataclass(frozen=True)
class TeleopConfig:
    slow: float
    normal: float
    reverse_max: float
    angular: float
    rate_hz: float
    deadzone: float
    pad_stale_s: float
    mux_mode_grace_s: float
    snapshot_stale_s: float
    required_components: tuple


@dataclass(frozen=True)
class GamepadMapping:
    verified: bool
    axis_forward: Optional[int]
    axis_turn: Optional[int]
    invert_forward: bool
    invert_turn: bool
    deadman: Optional[int]
    speed_toggle: Optional[int]


@dataclass(frozen=True)
class Command:
    linear: float
    angular: float
    deadman: bool
    reason: str = ""        # vide = commande normale ; sinon pourquoi la sortie est nulle


def _read(path) -> dict:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} : un dictionnaire YAML est attendu")
    return data


def load_teleop_config(path=None) -> TeleopConfig:
    d = _read(path or CONFIG_DIR / "teleop.yaml")
    sp = d.get("speeds") or {}
    cfg = TeleopConfig(
        slow=float(sp["slow"]), normal=float(sp["normal"]), reverse_max=float(sp["reverse_max"]),
        angular=float(sp["angular"]), rate_hz=float(d.get("publish_rate_hz", 20)),
        deadzone=float(d.get("deadzone", 0.12)), pad_stale_s=float(d.get("pad_stale_s", 0.3)),
        mux_mode_grace_s=float(d.get("mux_mode_grace_s", 1.5)),
        snapshot_stale_s=float(d.get("snapshot_stale_s", 3.0)),
        required_components=tuple(d.get("required_components") or ()))
    for name in ("slow", "normal", "reverse_max"):
        v = getattr(cfg, name)
        if not 0.0 < v <= HARD_MAX_LINEAR:
            raise ValueError(f"teleop.yaml : speeds.{name}={v} hors ]0, {HARD_MAX_LINEAR}]")
    if not 0.0 < cfg.angular <= HARD_MAX_ANGULAR:
        raise ValueError(f"teleop.yaml : speeds.angular hors ]0, {HARD_MAX_ANGULAR}]")
    if not 0.0 <= cfg.deadzone < 1.0 or not 1.0 <= cfg.rate_hz <= 50.0:
        raise ValueError("teleop.yaml : deadzone ∈ [0,1[ et publish_rate_hz ∈ [1,50]")
    return cfg


def _idx(v):
    if v is None:
        return None
    if isinstance(v, bool) or not isinstance(v, int) or v < 0:
        raise ValueError(f"gamepad.yaml : indice invalide {v!r} (entier >= 0 ou null)")
    return v


def load_gamepad_mapping(path=None) -> GamepadMapping:
    d = _read(path or CONFIG_DIR / "gamepad.yaml")
    ax, inv, bt = d.get("axes") or {}, d.get("invert") or {}, d.get("buttons") or {}
    return GamepadMapping(
        verified=bool(d.get("mapping_verified", False)),
        axis_forward=_idx(ax.get("forward")), axis_turn=_idx(ax.get("turn")),
        invert_forward=bool(inv.get("forward", False)), invert_turn=bool(inv.get("turn", False)),
        deadman=_idx(bt.get("deadman")), speed_toggle=_idx(bt.get("speed_toggle")))


def apply_deadzone(x: float, dz: float) -> float:
    if abs(x) <= dz:
        return 0.0
    return (abs(x) - dz) / (1.0 - dz) * (1.0 if x > 0 else -1.0)


class TeleopController:
    """Transforme l'état de la manette en commande (v, w). Sortie nulle par défaut."""

    def __init__(self, cfg: TeleopConfig, mapping: GamepadMapping) -> None:
        self.cfg, self.map = cfg, mapping

    def mapping_complete(self) -> bool:
        m = self.map
        return None not in (m.axis_forward, m.axis_turn, m.deadman)

    def compute(self, pad: Optional[PadState], speed_mode: str, now: float) -> Command:
        c, m = self.cfg, self.map
        if not self.mapping_complete():
            return Command(0.0, 0.0, False, "mapping de la manette incomplet (gamepad.yaml)")
        if pad is None or not pad.connected:
            return Command(0.0, 0.0, False, "manette absente")
        if now - pad.stamp > c.pad_stale_s:
            return Command(0.0, 0.0, False, "données de la manette périmées")
        try:
            deadman = bool(pad.buttons[m.deadman])
            f_raw, t_raw = float(pad.axes[m.axis_forward]), float(pad.axes[m.axis_turn])
        except (IndexError, TypeError, ValueError):
            return Command(0.0, 0.0, False, "indice d'axe/bouton absent sur cette manette")
        if not (math.isfinite(f_raw) and math.isfinite(t_raw)):
            return Command(0.0, 0.0, False, "valeur d'axe invalide")
        if not deadman:
            return Command(0.0, 0.0, False, "LB relâché")
        if m.invert_forward:
            f_raw = -f_raw
        if m.invert_turn:
            t_raw = -t_raw
        f, t = apply_deadzone(f_raw, c.deadzone), apply_deadzone(t_raw, c.deadzone)
        vmax = c.slow if speed_mode == "slow" else c.normal
        v = f * vmax if f >= 0 else f * min(vmax, c.reverse_max)
        w = t * c.angular
        return Command(max(-c.reverse_max, min(vmax, v)), max(-c.angular, min(c.angular, w)), True)


def manual_availability(s: Snapshot, cfg: TeleopConfig) -> tuple[bool, str]:
    """(ok, raison) : le pilotage manuel peut-il être ACTIVÉ ? Strict (aucune hypothèse)."""
    if not s.ros_available:
        return False, f"ROS 2 indisponible ({s.ros_error or 'rclpy introuvable'})"
    if not s.robot_connected:
        return False, "robot non connecté (aucun /robot_status récent)"
    if s.estop_active:
        return False, "arrêt d'urgence actif : réarmez d'abord"
    if not s.navigation_mode:
        return False, ("mode base/mapping détecté : le pilotage manuel exige le mode navigation "
                       "(navigation.launch.py)")
    for name in cfg.required_components:
        level, msg = s.components.get(name, (3, "absent"))
        if level != 0:
            return False, f"protection non opérationnelle : {name} ({msg})"
    if s.mux_mode is None:
        return False, "cmd_vel_mux_node ne répond pas (/cmd_vel_mode absent)"
    if s.safety_state not in OK_SAFETY:
        return False, f"état de sécurité : {s.safety_state or 'inconnu'}"
    if s.motor_status != "OK":
        return False, f"contrôleur moteur : {s.motor_status or 'inconnu'}"
    return True, ""


def session_must_end(s: Snapshot, cfg: TeleopConfig, session_age: float) -> str:
    """Raison pour laquelle une session ACTIVE doit s'arrêter ('' si elle peut continuer).
    Les blocages transitoires (hard-stop, Collision Monitor) ne terminent pas la session."""
    if not s.ros_available or not s.robot_connected:
        return "robot déconnecté"
    if s.estop_active:
        return "arrêt d'urgence"
    for name in cfg.required_components:
        if s.components.get(name, (3, ""))[0] >= 2:
            return f"protection en défaut : {name}"
    if session_age > cfg.mux_mode_grace_s and s.mux_mode != "MANUAL":
        return f"le multiplexeur n'est plus en MANUAL ({s.mux_mode or 'sans réponse'})"
    return ""

"""
Logique d'arbitrage AUTO / MANUAL / ESTOP du multiplexeur de commandes.

Module pur (aucun import ROS) : testable sans ROS. Le temps est fourni par l'appelant.

Règles :
  - ESTOP (/emergency_stop) : sortie nulle, session manuelle annulée, AUTO verrouillé
    jusqu'à ce que le flux automatique soit silencieux (aucune reprise d'ancien but).
  - MANUAL : uniquement tant que la session manuelle est vivante (heartbeat récent).
    Commande manuelle valide < manual_cmd_timeout, sinon commande nulle (jamais AUTO).
  - Fin de session manuelle (explicite, heartbeat perdu, ESTOP) : commande nulle puis
    AUTO_WAIT : l'automatique reste ignoré tant que le flux Nav2 n'a pas été silencieux
    auto_resume_gap secondes. Un ancien but ne peut donc pas reprendre.
  - AUTO : commande automatique valide < auto_cmd_timeout, sinon commande nulle.
"""

import math
from dataclasses import dataclass
from typing import NamedTuple, Optional

AUTO = 'AUTO'
AUTO_WAIT = 'AUTO_WAIT'
MANUAL = 'MANUAL'
ESTOP = 'ESTOP'


@dataclass
class MuxConfig:
    """Paramètres de l'arbitre (valeurs par défaut = robot_params.yaml)."""

    auto_max_linear: float = 0.30
    auto_max_angular: float = 1.0
    manual_max_forward: float = 0.20
    manual_max_reverse: float = 0.10
    manual_max_angular: float = 0.6
    manual_cmd_timeout: float = 0.3
    manual_session_timeout: float = 0.5
    auto_cmd_timeout: float = 0.5
    auto_resume_gap: float = 0.5
    manual_accel_linear: float = 0.4     # m/s^2 ; <= 0 : pas de rampe
    manual_accel_angular: float = 1.5    # rad/s^2


class MuxOutput(NamedTuple):
    """Sortie d'un cycle : vitesses et mode."""

    linear: float
    angular: float
    mode: str


def _finite(*values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _ramp(target, prev, accel, dt):
    """Rampe à la montée en valeur absolue ; arrêt et décélération immédiats."""
    if accel <= 0.0 or target == 0.0:
        return target
    if prev * target < 0.0:          # changement de sens : passer par zéro d'abord
        return 0.0
    if abs(target) <= abs(prev):
        return target
    step = accel * dt
    return prev + math.copysign(min(abs(target) - abs(prev), step), target)


class CmdVelArbiter:
    """Machine d'états de l'arbitrage."""

    def __init__(self, cfg: Optional[MuxConfig] = None):
        self.cfg = cfg or MuxConfig()
        self._auto = None            # (v, w, t)
        self._auto_rx = None         # dernier message auto reçu (même invalide)
        self._manual = None          # (v, w, t)
        self._estop = False
        self._session_flag = False
        self._session_t = None
        self._session_blocked = False
        self._in_manual = False
        self._gate = False           # True : AUTO ignoré (AUTO_WAIT)
        self._man_prev = (0.0, 0.0)

    # ---------- entrées ----------
    def on_auto(self, v, w, t):
        """Commande automatique (cmd_vel_smoothed). NaN/Inf rejetés."""
        self._auto_rx = t
        if not _finite(v, w):
            self._auto = None
            return
        c = self.cfg
        self._auto = (_clamp(v, -c.auto_max_linear, c.auto_max_linear),
                      _clamp(w, -c.auto_max_angular, c.auto_max_angular), t)

    def on_manual(self, v, w, t):
        """Commande manuelle (cmd_vel_teleop). NaN/Inf : la commande en cours est effacée."""
        if not _finite(v, w):
            self._manual = None
            return
        c = self.cfg
        self._manual = (_clamp(v, -c.manual_max_reverse, c.manual_max_forward),
                        _clamp(w, -c.manual_max_angular, c.manual_max_angular), t)

    def on_session(self, active, t):
        """Heartbeat de session manuelle (True répété) ou fin explicite (False)."""
        if active:
            self._session_t = t
            if not self._session_blocked:
                self._session_flag = True
        else:
            self._session_flag = False
            self._session_blocked = False

    def on_estop(self, flag):
        """Arrêt d'urgence logiciel (verrouillé tant que False n'est pas reçu)."""
        self._estop = bool(flag)

    # ---------- cycle ----------
    def step(self, t, dt) -> MuxOutput:
        """Calcule la sortie à l'instant t (dt = période du cycle)."""
        c = self.cfg
        if self._estop:
            self._enter_safe_state()
            self._session_blocked = True     # pas de reprise manuelle sur simple relâchement
            return MuxOutput(0.0, 0.0, ESTOP)

        if self._session_blocked and (self._session_t is None
                                      or t - self._session_t > c.manual_session_timeout):
            self._session_blocked = False    # le heartbeat a cessé : nouvelle activation possible

        alive = (self._session_flag and not self._session_blocked
                 and self._session_t is not None
                 and t - self._session_t <= c.manual_session_timeout)

        if alive:
            if not self._in_manual:
                self._in_manual = True
                self._man_prev = (0.0, 0.0)
                self._gate = True
            return self._manual_output(t, dt)

        if self._in_manual:                  # fin / perte de session : arrêt, pas de reprise
            self._enter_safe_state()
        self._session_flag = False

        if self._gate and (self._auto_rx is None or t - self._auto_rx >= c.auto_resume_gap):
            self._gate = False
        if self._gate:
            return MuxOutput(0.0, 0.0, AUTO_WAIT)
        if self._auto is not None and t - self._auto[2] <= c.auto_cmd_timeout:
            return MuxOutput(self._auto[0], self._auto[1], AUTO)
        return MuxOutput(0.0, 0.0, AUTO)

    def _manual_output(self, t, dt):
        c = self.cfg
        v = w = 0.0
        if self._manual is not None and t - self._manual[2] <= c.manual_cmd_timeout:
            v, w = self._manual[0], self._manual[1]
        v = _ramp(v, self._man_prev[0], c.manual_accel_linear, dt)
        w = _ramp(w, self._man_prev[1], c.manual_accel_angular, dt)
        self._man_prev = (v, w)
        return MuxOutput(v, w, MANUAL)

    def _enter_safe_state(self):
        """Annule toute commande mémorisée ; verrouille l'AUTO (AUTO_WAIT)."""
        self._in_manual = False
        self._session_flag = False
        self._manual = None
        self._man_prev = (0.0, 0.0)
        self._gate = True

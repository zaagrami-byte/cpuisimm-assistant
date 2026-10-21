"""Tests unitaires de l'arbitrage AUTO / MANUAL / ESTOP (sans ROS)."""

import math

from isimm_robot.cmd_vel_arbiter import (AUTO, AUTO_WAIT, ESTOP, MANUAL, CmdVelArbiter,
                                         MuxConfig)

DT = 0.05


def make(**kw):
    kw.setdefault('manual_accel_linear', 0.0)
    kw.setdefault('manual_accel_angular', 0.0)
    return CmdVelArbiter(MuxConfig(**kw))


def run(arb, t0, t1, auto=None, manual=None, session=None):
    """Simule des cycles de t0 à t1 en injectant les entrées à chaque cycle."""
    t, out = t0, None
    while t < t1 - 1e-9:
        if auto is not None:
            arb.on_auto(*auto, t)
        if manual is not None:
            arb.on_manual(*manual, t)
        if session is not None:
            arb.on_session(session, t)
        out = arb.step(t, DT)
        t += DT
    return out, t


def test_auto_passthrough_and_timeout():
    a = make()
    out, t = run(a, 0, 0.5, auto=(0.2, 0.1))
    assert (out.linear, out.angular, out.mode) == (0.2, 0.1, AUTO)
    out, _ = run(a, t, t + 0.7)                       # flux Nav2 arrêté
    assert (out.linear, out.angular, out.mode) == (0.0, 0.0, AUTO)


def test_auto_clamped_and_nan_rejected():
    a = make()
    out, t = run(a, 0, 0.3, auto=(5.0, -9.0))
    assert (out.linear, out.angular) == (0.30, -1.0)
    a.on_auto(math.nan, 0.0, t)
    assert a.step(t, DT).linear == 0.0
    a.on_auto(0.1, math.inf, t)
    assert a.step(t, DT).linear == 0.0


def test_manual_requires_session():
    a = make()
    out, _ = run(a, 0, 0.5, manual=(0.2, 0.0))        # pas de session : jamais de manuel
    assert out.mode == AUTO and out.linear == 0.0


def test_manual_priority_and_limits():
    a = make()
    out, _ = run(a, 0, 0.5, auto=(0.3, 0.0), manual=(0.5, 3.0), session=True)
    assert out.mode == MANUAL
    assert (out.linear, out.angular) == (0.20, 0.6)    # plafonds robot, auto ignoré
    out, _ = run(a, 0.5, 1.0, manual=(-0.5, 0.0), session=True)
    assert out.linear == -0.10                         # recul plafonné


def test_manual_command_timeout_gives_zero_not_auto():
    a = make()
    _, t = run(a, 0, 0.5, manual=(0.2, 0.0), session=True)
    out, _ = run(a, t, t + 0.4, auto=(0.25, 0.0), session=True)  # plus de commande manuelle
    assert out.mode == MANUAL and out.linear == 0.0


def test_manual_nan_clears_command():
    a = make()
    _, t = run(a, 0, 0.3, manual=(0.2, 0.0), session=True)
    a.on_manual(math.nan, 0.0, t)
    a.on_session(True, t)
    assert a.step(t, DT).linear == 0.0


def test_session_heartbeat_lost_stops_and_blocks_old_auto():
    a = make()
    _, t = run(a, 0, 0.5, manual=(0.2, 0.0), session=True)
    # heartbeat perdu pendant que Nav2 (ancien but / nouveau but IA) publie encore
    out, t = run(a, t, t + 2.0, auto=(0.25, 0.0))
    assert (out.linear, out.angular, out.mode) == (0.0, 0.0, AUTO_WAIT)
    # le flux Nav2 se tait -> AUTO redevient possible, mais sans reprise automatique
    out, t = run(a, t, t + 1.0)
    assert out.mode == AUTO and out.linear == 0.0
    out, t = run(a, t, t + 0.3, auto=(0.15, 0.0))     # nouveau but explicite
    assert out.mode == AUTO and out.linear == 0.15


def test_explicit_end_of_session_never_resumes_old_manual():
    a = make()
    _, t = run(a, 0, 0.5, manual=(0.2, 0.0), session=True)
    a.on_session(False, t)
    out, t = run(a, t, t + 1.0)
    assert out.linear == 0.0 and out.mode == AUTO
    out, _ = run(a, t, t + 0.5, session=True)          # réactivation : ancienne commande effacée
    assert out.mode == MANUAL and out.linear == 0.0


def test_zero_output_when_leaving_manual_even_with_auto_active():
    a = make()
    _, t = run(a, 0, 0.5, manual=(0.2, 0.0), session=True)
    a.on_session(False, t)
    a.on_auto(0.25, 0.0, t)
    out = a.step(t, DT)
    assert out.linear == 0.0 and out.mode == AUTO_WAIT


def test_estop_blocks_everything_and_requires_reactivation():
    a = make()
    _, t = run(a, 0, 0.5, manual=(0.2, 0.0), session=True)
    a.on_estop(True)
    out, t = run(a, t, t + 0.5, auto=(0.3, 0.0), manual=(0.2, 0.0), session=True)
    assert (out.linear, out.angular, out.mode) == (0.0, 0.0, ESTOP)
    a.on_estop(False)                                  # réarmement, UI poursuit son heartbeat
    out, t = run(a, t, t + 0.4, auto=(0.3, 0.0), manual=(0.2, 0.0), session=True)
    assert out.mode == AUTO_WAIT and out.linear == 0.0   # aucun mouvement après réarmement
    out, t = run(a, t, t + 1.0)                           # heartbeat arrêté
    assert out.mode == AUTO and out.linear == 0.0
    out, _ = run(a, t, t + 0.5, manual=(0.1, 0.0), session=True)
    assert out.mode == MANUAL and out.linear == 0.1       # réactivation explicite


def test_estop_then_auto_stream_silent_resumes_only_new_goal():
    a = make()
    a.on_estop(True)
    a.step(0.0, DT)
    a.on_estop(False)
    out, t = run(a, 0.05, 0.6, auto=(0.2, 0.0))
    assert out.mode == AUTO_WAIT and out.linear == 0.0
    out, t = run(a, t, t + 0.8)
    assert out.mode == AUTO


def test_manual_ramp_up_and_instant_stop():
    a = CmdVelArbiter(MuxConfig(manual_accel_linear=0.4))
    t, vs = 0.0, []
    for _ in range(10):
        a.on_manual(0.2, 0.0, t)
        a.on_session(True, t)
        vs.append(a.step(t, DT).linear)
        t += DT
    assert abs(vs[0] - 0.02) < 1e-9 and all(b >= a_ for a_, b in zip(vs, vs[1:]))
    assert vs[4] < 0.2 and abs(vs[-1] - 0.2) < 1e-9   # montée progressive puis plateau
    a.on_manual(0.0, 0.0, t)
    a.on_session(True, t)
    assert a.step(t, DT).linear == 0.0                  # arrêt sans rampe


def test_manual_direction_reversal_passes_through_zero():
    a = CmdVelArbiter(MuxConfig(manual_accel_linear=0.0))
    a.on_manual(0.2, 0.0, 0.0)
    a.on_session(True, 0.0)
    assert a.step(0.0, DT).linear == 0.2
    a.cfg.manual_accel_linear = 0.4
    a.on_manual(-0.1, 0.0, 0.05)
    a.on_session(True, 0.05)
    assert a.step(0.05, DT).linear == 0.0

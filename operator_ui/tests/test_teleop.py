import pytest

from operator_ui.model import PadState, Snapshot
from operator_ui.teleop import (GamepadMapping, TeleopConfig, TeleopController, apply_deadzone,
                                load_gamepad_mapping, load_teleop_config, manual_availability,
                                session_must_end)

CFG = TeleopConfig(slow=0.10, normal=0.20, reverse_max=0.10, angular=0.6, rate_hz=20, deadzone=0.12,
                   pad_stale_s=0.3, mux_mode_grace_s=1.5, snapshot_stale_s=3.0,
                   required_components=("safety_node", "controller_server", "cmd_vel_mux"))
MAP = GamepadMapping(True, 1, 3, True, True, 4, None)


def pad(fwd=0.0, turn=0.0, lb=True, t=100.0, connected=True):
    axes = (0.0, fwd, 0.0, turn, 0.0, 0.0)
    buttons = (False, False, False, False, lb, False)
    return PadState(connected=connected, name="Test", axes=axes, buttons=buttons, stamp=t)


def ctrl(mapping=MAP):
    return TeleopController(CFG, mapping)


def test_deadzone():
    assert apply_deadzone(0.1, 0.12) == 0.0 and apply_deadzone(-0.12, 0.12) == 0.0
    assert apply_deadzone(1.0, 0.12) == pytest.approx(1.0)


def test_forward_speeds_and_inversion():
    c = ctrl()
    cmd = c.compute(pad(fwd=-1.0), "normal", 100.0)     # stick vers le haut = -1 -> avant
    assert cmd.linear == pytest.approx(0.20) and cmd.deadman
    assert c.compute(pad(fwd=-1.0), "slow", 100.0).linear == pytest.approx(0.10)


def test_reverse_capped_at_010_even_in_normal():
    cmd = ctrl().compute(pad(fwd=1.0), "normal", 100.0)
    assert cmd.linear == pytest.approx(-0.10)


def test_turn_direction_and_limit():
    cmd = ctrl().compute(pad(turn=1.0), "normal", 100.0)  # stick à droite -> rotation horaire (w<0)
    assert cmd.angular == pytest.approx(-0.6)


def test_inside_deadzone_is_zero():
    cmd = ctrl().compute(pad(fwd=-0.1, turn=0.1), "normal", 100.0)
    assert (cmd.linear, cmd.angular) == (0.0, 0.0) and cmd.deadman


def test_deadman_released_is_zero():
    cmd = ctrl().compute(pad(fwd=-1.0, turn=1.0, lb=False), "normal", 100.0)
    assert (cmd.linear, cmd.angular, cmd.deadman) == (0.0, 0.0, False) and cmd.reason


@pytest.mark.parametrize("p,now", [(None, 100.0), (PadState(), 100.0), (pad(fwd=-1.0), 100.5)])
def test_pad_lost_or_stale_is_zero(p, now):
    cmd = ctrl().compute(p, "normal", now)
    assert (cmd.linear, cmd.angular) == (0.0, 0.0) and cmd.reason


def test_nan_axis_is_zero():
    cmd = ctrl().compute(pad(fwd=float("nan")), "normal", 100.0)
    assert (cmd.linear, cmd.angular) == (0.0, 0.0)


def test_incomplete_mapping_is_zero_never_invented():
    m = GamepadMapping(False, None, 3, True, True, 4, None)
    c = ctrl(m)
    assert not c.mapping_complete()
    assert c.compute(pad(fwd=-1.0), "normal", 100.0).linear == 0.0


def test_out_of_range_index_is_zero():
    m = GamepadMapping(True, 9, 3, True, True, 4, None)
    assert ctrl(m).compute(pad(fwd=-1.0), "normal", 100.0).linear == 0.0


# ---------------------------------------------------------------- disponibilité
def good():
    comps = {"safety_node": (0, "OK"), "controller_server": (0, "active"), "cmd_vel_mux": (0, "OK")}
    return Snapshot(ros_available=True, robot_connected=True, components=comps, safety_state="OK",
                    motor_status="OK", mux_mode="AUTO")


def test_available_when_everything_ok():
    assert manual_availability(good(), CFG) == (True, "")


def test_unavailable_without_ros_or_robot():
    ok, why = manual_availability(Snapshot(), CFG)
    assert not ok and "ROS 2" in why
    ok, why = manual_availability(Snapshot(ros_available=True), CFG)
    assert not ok and "non connecté" in why


def test_unavailable_in_base_or_mapping_mode():
    s = good()
    comps = dict(s.components)
    del comps["controller_server"]
    s2 = Snapshot(**{**s.__dict__, "components": comps})
    ok, why = manual_availability(s2, CFG)
    assert not ok and "navigation" in why


@pytest.mark.parametrize("field,value", [
    ("safety_state", "EMERGENCY_STOP"), ("motor_status", "ARDUINO_ESTOP"), ("mux_mode", "ESTOP")])
def test_unavailable_during_estop(field, value):
    s = Snapshot(**{**good().__dict__, field: value})
    ok, why = manual_availability(s, CFG)
    assert not ok and "urgence" in why


def test_unavailable_when_protection_missing_or_degraded():
    s = good()
    comps = dict(s.components)
    comps["cmd_vel_mux"] = (2, "STALE")
    ok, why = manual_availability(Snapshot(**{**s.__dict__, "components": comps}), CFG)
    assert not ok and "cmd_vel_mux" in why
    ok, _ = manual_availability(Snapshot(**{**s.__dict__, "mux_mode": None}), CFG)
    assert not ok
    ok, _ = manual_availability(Snapshot(**{**s.__dict__, "safety_state": "HARD_STOP_FRONT"}), CFG)
    assert not ok


def test_session_must_end_rules():
    s = good()
    s_man = Snapshot(**{**s.__dict__, "mux_mode": "MANUAL"})
    assert session_must_end(s_man, CFG, 5.0) == ""
    assert session_must_end(Snapshot(), CFG, 5.0)                          # robot déconnecté
    assert session_must_end(Snapshot(**{**s_man.__dict__, "safety_state": "EMERGENCY_STOP"}), CFG, 5.0)
    assert session_must_end(s, CFG, 0.5) == ""                             # délai de grâce du mux
    assert "multiplexeur" in session_must_end(s, CFG, 5.0)                 # mux sorti de MANUAL
    blocked = Snapshot(**{**s_man.__dict__, "safety_state": "HARD_STOP_FRONT"})
    assert session_must_end(blocked, CFG, 5.0) == ""                        # blocage transitoire toléré


# ---------------------------------------------------------------- configuration
def test_shipped_config_loads_with_expected_limits():
    c = load_teleop_config()
    assert (c.slow, c.normal, c.reverse_max, c.angular, c.rate_hz, c.deadzone) == (
        0.10, 0.20, 0.10, 0.6, 20, 0.12)
    m = load_gamepad_mapping()
    assert m.deadman == 4 and not m.verified


def test_config_rejects_unsafe_values(tmp_path):
    f = tmp_path / "t.yaml"
    f.write_text("speeds: {slow: 0.1, normal: 0.9, reverse_max: 0.1, angular: 0.6}\n")
    with pytest.raises(ValueError):
        load_teleop_config(f)
    g = tmp_path / "g.yaml"
    g.write_text("axes: {forward: -1, turn: 3}\n")
    with pytest.raises(ValueError):
        load_gamepad_mapping(g)

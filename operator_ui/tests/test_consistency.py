"""Cohérence topics / configuration entre ros2_ws, scripts et operator_ui (fichiers réels du dépôt)."""
import re
from pathlib import Path

import yaml

from operator_ui import ros_bridge as rb
from operator_ui.teleop import load_teleop_config

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT / "ros2_ws/src/isimm_robot"


def load(name):
    return yaml.safe_load((PKG / "config" / name).read_text())


def test_collision_monitor_reads_mux_output_and_still_publishes_cmd_vel():
    cm = load("nav2_params.yaml")["collision_monitor"]["ros__parameters"]
    assert cm["cmd_vel_in_topic"] == "cmd_vel_mux" and cm["cmd_vel_out_topic"] == "cmd_vel"


def test_mux_node_topics_match_ui_constants():
    src = (PKG / "isimm_robot/cmd_vel_mux_node.py").read_text()
    for topic in (rb.T_TELEOP, rb.T_SESSION, rb.T_ESTOP, rb.T_MODE):
        assert f"'{topic.lstrip('/')}'" in src, topic
    assert "'cmd_vel_smoothed'" in src and "'cmd_vel_mux'" in src
    assert "'cmd_vel'" not in src           # le mux ne publie jamais /cmd_vel


def test_mux_is_only_publisher_path_and_ui_never_uses_cmd_vel():
    ui_src = "".join(p.read_text() for p in (ROOT / "operator_ui").glob("*.py"))
    assert not re.search(r"""["']/cmd_vel["']""", ui_src)
    assert rb.T_TELEOP == "/cmd_vel_teleop"


def test_launch_and_setup_register_mux():
    launch = (PKG / "launch/navigation.launch.py").read_text()
    assert "cmd_vel_mux_node" in launch and "robot_params" in launch
    assert "cmd_vel_mux_node = isimm_robot.cmd_vel_mux_node:main" in (PKG / "setup.py").read_text()


def test_safety_and_collision_monitor_nodes_unchanged_in_chain():
    launch = (PKG / "launch/navigation.launch.py").read_text()
    assert "collision_monitor" in launch
    robot = (PKG / "launch/robot.launch.py").read_text()
    assert "safety_node" in robot and "remappings" not in robot      # safety_node : /cmd_vel sans remap


def test_manual_limits_consistent_between_ui_and_robot():
    mux = load("robot_params.yaml")["cmd_vel_mux_node"]["ros__parameters"]
    ui = load_teleop_config()
    assert ui.normal <= mux["manual_max_forward"] and ui.slow <= ui.normal
    assert ui.reverse_max <= mux["manual_max_reverse"] and ui.angular <= mux["manual_max_angular"]
    assert mux["manual_cmd_timeout"] == 0.3 and mux["publish_rate"] == 20.0
    rp = load("robot_params.yaml")
    assert mux["manual_max_forward"] <= rp["safety_node"]["ros__parameters"]["max_linear_speed"]
    assert mux["manual_max_angular"] <= rp["motor_controller_node"]["ros__parameters"]["max_angular_speed"]
    assert mux["manual_max_forward"] <= rp["motor_controller_node"]["ros__parameters"]["max_linear_speed"]
    # heartbeat UI (10 Hz) nettement plus rapide que le timeout de session du mux
    assert mux["manual_session_timeout"] >= 0.3


def test_required_components_exist_in_status_node_names():
    src = (PKG / "isimm_robot/robot_status_node.py").read_text()
    for name in load_teleop_config().required_components:
        assert name in src or name in ("cmd_vel_mux",), name
    assert "'cmd_vel_mux'" in src


def test_check_robot_script_covers_new_chain():
    sh = (ROOT / "scripts/check_robot.sh").read_text()
    for token in ("/cmd_vel_mux", "/cmd_vel_teleop", "cmd_vel_mux_node", "/cmd_vel_mode"):
        assert token in sh

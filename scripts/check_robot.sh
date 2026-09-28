 #!/usr/bin/env bash
# Diagnostic ISIMM ROBOT — LECTURE SEULE (ne publie rien, ne modifie rien).
# Usage : scripts/check_robot.sh [base|mapping|navigation]   (défaut : auto)
# Prérequis : source /opt/ros/jazzy/setup.bash && source ~/isimm/ros2_ws/install/setup.bash
set -u
export PYTHONUNBUFFERED=1
PASS=0; WARN=0; FAIL=0
pass() { echo -e "\e[32m[PASS]\e[0m $*"; PASS=$((PASS+1)); }
warn() { echo -e "\e[33m[WARN]\e[0m $*"; WARN=$((WARN+1)); }
fail() { echo -e "\e[31m[FAIL]\e[0m $*"; FAIL=$((FAIL+1)); }

command -v ros2 >/dev/null || { echo "ros2 introuvable : sourcez ROS 2 Jazzy"; exit 2; }

echo "== Environnement =="
[ "${ROS_DISTRO:-}" = "jazzy" ] && pass "ROS_DISTRO=jazzy" || fail "ROS_DISTRO='${ROS_DISTRO:-}' (attendu jazzy)"
[ -n "${ROS_DOMAIN_ID:-}" ] && pass "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" || warn "ROS_DOMAIN_ID non défini (0 par défaut) — identique sur Pi et PC ?"

echo "== Périphériques série =="
for d in isimm_arduino isimm_rplidar; do
  [ -e "/dev/$d" ] && pass "/dev/$d -> $(readlink -f /dev/$d)" || fail "/dev/$d absent (udev / câble / port USB)"
done
[ -e /dev/isimm_esp32 ] && pass "/dev/isimm_esp32 présent" || warn "/dev/isimm_esp32 absent (IMU indisponible, non bloquant)"

echo "== Nœuds =="
NODES=$(ros2 node list 2>/dev/null | sort)
DUP=$(echo "$NODES" | uniq -d)
[ -z "$DUP" ] && pass "aucun nom de nœud dupliqué" || fail "nœuds dupliqués : $(echo $DUP)"
has() { echo "$NODES" | grep -qx "$1"; }

MODE="${1:-auto}"
if [ "$MODE" = "auto" ]; then
  if has /controller_server; then MODE=navigation
  elif has /slam_toolbox; then MODE=mapping
  else MODE=base; fi
fi
echo "   mode = $MODE"

for n in /rplidar_node /CLaserOdometry2DNode /safety_node /motor_controller_node /robot_status_node /robot_state_publisher; do
  has "$n" && pass "nœud $n" || fail "nœud $n absent"
done
has /esp32_sensor_node && pass "nœud /esp32_sensor_node" || warn "nœud /esp32_sensor_node absent"
if [ "$MODE" != base ]; then has /slam_toolbox && pass "nœud /slam_toolbox" || fail "/slam_toolbox absent"; fi
if [ "$MODE" = navigation ]; then
  for n in controller_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother collision_monitor; do
    has "/$n" && pass "nœud /$n" || fail "nœud /$n absent"
  done
fi

echo "== Lifecycle =="
LC=""
[ "$MODE" != base ] && LC="slam_toolbox"
[ "$MODE" = navigation ] && LC="$LC controller_server planner_server behavior_server velocity_smoother collision_monitor bt_navigator waypoint_follower"
for n in $LC; do
  st=$(timeout 5 ros2 lifecycle get "/$n" 2>/dev/null | head -1)
  echo "$st" | grep -q "active" && pass "/$n : $st" || fail "/$n : ${st:-injoignable}"
done

echo "== Fréquences topics =="
hz() { # topic minHz level(fail|warn)
  local r
  r=$(timeout 8 ros2 topic hz "$1" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
  if [ -z "$r" ]; then [ "$3" = fail ] && fail "$1 : aucun message" || warn "$1 : aucun message"; return; fi
  if awk "BEGIN{exit !($r >= $2)}"; then pass "$1 : ${r} Hz (>= $2)"; else
    [ "$3" = fail ] && fail "$1 : ${r} Hz (< $2)" || warn "$1 : ${r} Hz (< $2)"; fi
}
hz /scan 4 fail
hz /odom 4 fail
hz /safe_cmd_vel 10 fail
hz /imu 5 warn

echo "== TF =="
tf() {
  local r
  r=$(timeout 6 stdbuf -oL ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1 | grep -m1 "At time")
  [ -n "$r" ] && pass "TF $1 -> $2" || { [ "$3" = fail ] && fail "TF $1 -> $2 absent" || warn "TF $1 -> $2 absent"; }
}
tf odom base_link fail
tf base_link laser_link fail
[ "$MODE" != base ] && tf map odom fail
TFPUB=$(timeout 6 ros2 topic info /tf -v 2>/dev/null | grep "Node name" | awk '{print $3}' | sort | uniq -c)
echo "   publishers /tf : $(echo $TFPUB)"
echo "$TFPUB" | awk '$1>1{f=1} END{exit f}' || fail "un publisher /tf en double"
echo "$TFPUB" | grep -Eqv "CLaserOdometry2DNode|slam_toolbox|^$" && warn "publisher /tf inattendu (voir liste)" || pass "publishers /tf attendus (RF2O, slam_toolbox)"

echo "== Chaîne cmd_vel / unicité =="
pubs() { timeout 5 ros2 topic info "$1" 2>/dev/null | awk '/Publisher count/{print $3}'; }
subs() { timeout 5 ros2 topic info "$1" 2>/dev/null | awk '/Subscription count/{print $3}'; }
chk() { [ "${2:-x}" = "$3" ] && pass "$1 = $3" || fail "$1 = ${2:-?} (attendu $3)"; }
chk "publishers /scan" "$(pubs /scan)" 1
chk "publishers /odom" "$(pubs /odom)" 1
chk "publishers /safe_cmd_vel" "$(pubs /safe_cmd_vel)" 1
if [ "$MODE" = navigation ]; then
  chk "publishers /cmd_vel_nav (controller+behavior)" "$(pubs /cmd_vel_nav)" 2
  chk "publishers /cmd_vel_smoothed" "$(pubs /cmd_vel_smoothed)" 1
  chk "publishers /cmd_vel (collision_monitor seul)" "$(pubs /cmd_vel)" 1
  chk "subscribers /cmd_vel (safety_node seul)" "$(subs /cmd_vel)" 1
fi

echo "== Moteurs / sécurité =="
ms=$(timeout 4 ros2 topic echo --once /motor_status --field data 2>/dev/null | head -1)
[ "$ms" = "OK" ] && pass "motor_status = OK (série + Arduino)" || fail "motor_status = '${ms:-aucun}'"
ss=$(timeout 4 ros2 topic echo --once /safety_state --field data 2>/dev/null | head -1)
case "$ss" in OK) pass "safety_state = OK";; NO_CMD_YET|CMD_TIMEOUT) pass "safety_state = $ss (repos normal)";;
  "") fail "safety_state : aucun message";; *) warn "safety_state = $ss";; esac

echo "== use_sim_time =="
for n in safety_node motor_controller_node robot_state_publisher; do
  v=$(timeout 4 ros2 param get "/$n" use_sim_time 2>/dev/null | awk '{print $NF}')
  [ "$v" = "False" ] && pass "/$n use_sim_time=False" || warn "/$n use_sim_time='${v:-?}'"
done

echo
echo "RÉSULTAT : $PASS PASS, $WARN WARN, $FAIL FAIL"
[ "$FAIL" -eq 0 ]

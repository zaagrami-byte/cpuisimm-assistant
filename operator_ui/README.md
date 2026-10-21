# operator_ui — Interface opérateur du robot ISIMM (PC)

Supervision (3 onglets : État, LiDAR, Pilotage manuel) et pilotage manuel à la manette.
Ne touche ni `ai/` ni `/cmd_vel` : les commandes manuelles passent par `/cmd_vel_teleop`
→ `cmd_vel_mux_node` (Pi) → Collision Monitor → `safety_node` → moteurs.

## Chaîne de commande (mode navigation)
```
velocity_smoother → /cmd_vel_smoothed ─┐
interface PC      → /cmd_vel_teleop  ──┴→ cmd_vel_mux_node → /cmd_vel_mux → collision_monitor
                                                → /cmd_vel → safety_node → /safe_cmd_vel → motor_controller → Arduino
```
Topics ajoutés : `/cmd_vel_mode` (String : AUTO | AUTO_WAIT | MANUAL | ESTOP), `/manual_session`
(Bool, heartbeat 10 Hz de l'interface). Voir `ros2_ws/.../cmd_vel_mux_node.py`.

## Installation (PC, Ubuntu 24.04 + ROS 2 Jazzy)
```bash
source /opt/ros/jazzy/setup.bash
cd ~/isimm                      # racine du dépôt
python3 -m venv --system-site-packages operator_ui/venv
source operator_ui/venv/bin/activate
pip install -r operator_ui/requirements.txt
```
## Lancement
```bash
source /opt/ros/jazzy/setup.bash && source operator_ui/venv/bin/activate
export ROS_DOMAIN_ID=0          # identique au Pi
python -m operator_ui
python -m operator_ui --check-gamepad     # vérifier axes/boutons sans ROS
```
## Configuration
- `config/teleop.yaml` : vitesses (0,10 / 0,20 / recul 0,10 m/s, 0,6 rad/s), zone morte 0,12, 20 Hz.
  Les plafonds réels sont imposés côté robot (`robot_params.yaml` → `cmd_vel_mux_node`).
- `config/gamepad.yaml` : mapping de la manette (indices CANDIDATS, à vérifier). Tant que
  `mapping_verified: false`, il faut cocher « J'ai vérifié le mapping » à chaque session.

## Tests
```bash
QT_QPA_PLATFORM=offscreen python -m pytest operator_ui/tests -q
cd ros2_ws/src/isimm_robot && PYTHONPATH=. python -m pytest test/test_cmd_vel_arbiter.py -q
```

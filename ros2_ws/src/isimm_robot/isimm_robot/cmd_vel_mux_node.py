"""
Cmd Vel Mux Node — ISIMM ROBOT.

Chaîne (mode navigation) :
  velocity_smoother -> /cmd_vel_smoothed ----\
                                              +-> [cmd_vel_mux_node] -> /cmd_vel_mux
  interface PC      -> /cmd_vel_teleop  ------/        -> collision_monitor -> /cmd_vel
                                                       -> safety_node -> /safe_cmd_vel

Seul arbitre entre commandes automatiques et manuelles. NE publie PAS /cmd_vel : la sortie
passe toujours par le Collision Monitor puis le safety_node.

Entrées :
  /cmd_vel_smoothed   (Twist)  automatique (Nav2)
  /cmd_vel_teleop     (Twist)  manuel (interface opérateur, 20 Hz)
  /manual_session     (Bool)   heartbeat True tant que le pilotage manuel est activé
  /emergency_stop     (Bool)   arrêt d'urgence logiciel (verrouillé)
Sorties :
  /cmd_vel_mux        (Twist)  20 Hz en continu (zéro si rien de valide)
  /cmd_vel_mode       (String) AUTO | AUTO_WAIT | MANUAL | ESTOP
      AUTO_WAIT : sortie nulle, AUTO ignoré après un pilotage manuel ou un ESTOP, jusqu'à ce
      que le flux Nav2 soit silencieux (aucune reprise d'un ancien but).
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from isimm_robot.cmd_vel_arbiter import CmdVelArbiter, MuxConfig


class CmdVelMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux_node')
        defaults = MuxConfig()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, float(value))
        self.declare_parameter('publish_rate', 20.0)
        cfg = MuxConfig(**{n: float(self.get_parameter(n).value) for n in vars(defaults)})
        rate = float(self.get_parameter('publish_rate').value)
        self.dt = 1.0 / rate
        self.arb = CmdVelArbiter(cfg)

        self.create_subscription(Twist, 'cmd_vel_smoothed', self._auto_cb, 10)
        self.create_subscription(Twist, 'cmd_vel_teleop', self._manual_cb, 10)
        self.create_subscription(Bool, 'manual_session', self._session_cb, 10)
        self.create_subscription(Bool, 'emergency_stop', self._estop_cb, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel_mux', 10)
        self.mode_pub = self.create_publisher(String, 'cmd_vel_mode', 10)
        self._mode = None
        self._mode_pub_t = 0.0
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'cmd_vel_mux actif : cmd_vel_smoothed + cmd_vel_teleop -> cmd_vel_mux '
            f'({rate:.0f} Hz, manuel max {cfg.manual_max_forward}/-{cfg.manual_max_reverse} m/s, '
            f'{cfg.manual_max_angular} rad/s, timeout manuel {cfg.manual_cmd_timeout} s)')

    def _auto_cb(self, m: Twist):
        self.arb.on_auto(m.linear.x, m.angular.z, time.monotonic())

    def _manual_cb(self, m: Twist):
        self.arb.on_manual(m.linear.x, m.angular.z, time.monotonic())

    def _session_cb(self, m: Bool):
        self.arb.on_session(m.data, time.monotonic())

    def _estop_cb(self, m: Bool):
        self.arb.on_estop(m.data)

    def _tick(self):
        now = time.monotonic()
        out = self.arb.step(now, self.dt)
        msg = Twist()
        msg.linear.x = float(out.linear)
        msg.angular.z = float(out.angular)
        self.pub.publish(msg)
        if out.mode != self._mode or now - self._mode_pub_t > 0.5:
            if out.mode != self._mode:
                self.get_logger().info(f'Mode de commande : {self._mode} -> {out.mode}')
            self._mode, self._mode_pub_t = out.mode, now
            self.mode_pub.publish(String(data=out.mode))

    def destroy_node(self):
        try:
            self.pub.publish(Twist())     # STOP final
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

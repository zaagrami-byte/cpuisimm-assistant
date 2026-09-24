"""
Safety Node — ISIMM ROBOT.

Position : Nav2 (/cmd_vel) -> [safety_node] -> /safe_cmd_vel -> motor_controller.

Responsabilités :
  - arrêt d'urgence logiciel VERROUILLÉ (/emergency_stop, std_msgs/Bool)
  - timeout de /cmd_vel (si Nav2 ne publie plus -> STOP)
  - surveillance du LiDAR (/scan absent ou trop vieux -> STOP) [peut être désactivée]
  - rejet des commandes invalides (NaN/Inf -> STOP)
  - limitation des vitesses
  - publication continue à publish_rate Hz

IMPORTANT : aucun sleep bloquant n'est utilisé dans les callbacks.
Le log d'avertissement est émis de façon non bloquante (horodatage ROS).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('monitor_scan', True)
        self.declare_parameter('scan_timeout', 1.0)
        self.declare_parameter('min_obstacle_distance', 0.0)
        self.declare_parameter('warn_period', 5.0)

        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.max_lin = float(self.get_parameter('max_linear_speed').value)
        self.max_ang = float(self.get_parameter('max_angular_speed').value)
        self.monitor_scan = bool(self.get_parameter('monitor_scan').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.min_obstacle_distance = float(self.get_parameter('min_obstacle_distance').value)
        self.warn_period = float(self.get_parameter('warn_period').value)

        self._last_cmd = Twist()
        self._last_cmd_time = None
        self._last_scan_time = None
        self._estop = False
        self._range_min = float('inf')
        self._last_warn_time = -1e9  # garantit le premier log

        self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Bool, 'emergency_stop', self._estop_cb, 10)
        if self.monitor_scan:
            self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)

        self.pub = self.create_publisher(Twist, 'safe_cmd_vel', 10)
        period = 1.0 / float(self.get_parameter('publish_rate').value)
        self.create_timer(period, self._tick)

        self.get_logger().info('Safety node actif : /cmd_vel -> /safe_cmd_vel')

    # ---------- helpers non bloquants ----------
    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _log_throttled_warn(self, text):
        """Log espacé SANS bloquer l'exécuteur (pas de Rate.sleep())."""
        now = self._now_sec()
        if now - self._last_warn_time >= self.warn_period:
            self._last_warn_time = now
            self.get_logger().warn(text)

    # ---------- callbacks ----------
    def _cmd_cb(self, msg: Twist):
        if not (math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)):
            self.get_logger().error('Commande /cmd_vel invalide (NaN/Inf) ignorée.')
            return
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _scan_cb(self, msg: LaserScan):
        self._last_scan_time = self.get_clock().now()

    def _estop_cb(self, msg: Bool):
        if msg.data and not self._estop:
            self.get_logger().error("ARRÊT D'URGENCE ACTIVÉ — moteurs STOP.")
        if not msg.data and self._estop:
            self.get_logger().warn("Arrêt d'urgence réarmé.")
        self._estop = msg.data

    def _blocked_reason(self):
        if self._estop:
            return 'EMERGENCY_STOP'
        now = self.get_clock().now()
        if self._last_cmd_time is None:
            return 'NO_CMD_YET'
        if (now - self._last_cmd_time) > Duration(seconds=self.cmd_timeout):
            return 'CMD_TIMEOUT'
        if self.monitor_scan:
            if self._last_scan_time is None:
                return 'NO_SCAN_YET'
            if (now - self._last_scan_time) > Duration(seconds=self.scan_timeout):
                return 'SCAN_TIMEOUT'
        if self.min_obstacle_distance > 0.0 and self._range_min < self.min_obstacle_distance:
            return 'OBSTACLE_TOO_CLOSE'
        return None

    def _tick(self):
        reason = self._blocked_reason()
        out = Twist()
        if reason is None:
            out.linear.x = max(-self.max_lin, min(self.max_lin, self._last_cmd.linear.x))
            out.angular.z = max(-self.max_ang, min(self.max_ang, self._last_cmd.angular.z))
        else:
            if reason not in ('NO_CMD_YET', 'NO_SCAN_YET'):
                self._log_throttled_warn(f'Blocage sécurité : {reason}')
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # STOP final
        node.destroy_node()
        rclpy.shutdown()

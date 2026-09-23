"""
Safety Node — ISIMM ROBOT.

Position : Nav2 (/cmd_vel) -> [safety_node] -> /safe_cmd_vel -> motor_controller.

Responsabilités :
  - arrêt d'urgence logiciel VERROUILLÉ (/emergency_stop, std_msgs/Bool)
  - timeout de /cmd_vel (si Nav2 ne publie plus -> STOP)
  - surveillance du LiDAR (/scan trop vieux -> STOP) [peut être désactivée]
  - limitation des vitesses
  - publication continue à publish_rate Hz (le moteur doit toujours
    recevoir quelque chose, même un STOP répété)

Extension ultrasons prévue : brancher /range/* -> ajouter une condition
dans _check_sensors() sans toucher au reste.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('cmd_timeout', 0.5)          # s sans cmd_vel -> STOP
        self.declare_parameter('max_linear_speed', 0.5)     # m/s
        self.declare_parameter('max_angular_speed', 1.5)    # rad/s
        self.declare_parameter('publish_rate', 20.0)        # Hz
        self.declare_parameter('monitor_scan', True)
        self.declare_parameter('scan_timeout', 1.0)         # s
        self.declare_parameter('min_obstacle_distance', 0.0)  # m, 0 = désactivé (futur ultrasons)

        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.max_lin = float(self.get_parameter('max_linear_speed').value)
        self.max_ang = float(self.get_parameter('max_angular_speed').value)
        self.monitor_scan = bool(self.get_parameter('monitor_scan').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.min_obstacle_distance = float(self.get_parameter('min_obstacle_distance').value)

        self._last_cmd = Twist()
        self._last_cmd_time = None
        self._last_scan_time = None
        self._estop = False
        self._range_min = float('inf')

        self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Bool, 'emergency_stop', self._estop_cb, 10)
        if self.monitor_scan:
            self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        # ultrasons futurs : self.create_subscription(Range, 'range/front_left', ...)

        self.pub = self.create_publisher(Twist, 'safe_cmd_vel', 10)
        period = 1.0 / float(self.get_parameter('publish_rate').value)
        self.create_timer(period, self._tick)
        self._rate_limited_log = self.create_rate(0.2)  # logs espacés

        self.get_logger().info('Safety node actif : /cmd_vel -> /safe_cmd_vel')

    def _cmd_cb(self, msg: Twist):
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _scan_cb(self, msg: LaserScan):
        self._last_scan_time = self.get_clock().now()

    def _estop_cb(self, msg: Bool):
        if msg.data and not self._estop:
            self.get_logger().error('ARRÊT D\'URGENCE ACTIVÉ — moteurs STOP.')
        if not msg.data and self._estop:
            self.get_logger().warn('Arrêt d\'urgence réarmé.')
        self._estop = msg.data

    def _blocked_reason(self):
        """Retourne la raison du blocage ou None."""
        if self._estop:
            return 'EMERGENCY_STOP'
        now = self.get_clock().now()
        if self._last_cmd_time is None:
            return 'NO_CMD_YET'
        if (now - self._last_cmd_time) > Duration(seconds=self.cmd_timeout):
            return 'CMD_TIMEOUT'
        if self.monitor_scan and self._last_scan_time is not None:
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
            self._rate_limited_log.sleep()  # écarte les logs
            if reason != 'NO_CMD_YET':
                self.get_logger().warn(f'Blocage sécurité : {reason}')
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

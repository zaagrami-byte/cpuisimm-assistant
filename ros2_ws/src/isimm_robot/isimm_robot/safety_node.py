"""
Safety Node — ISIMM ROBOT.

Position : collision_monitor (/cmd_vel) -> [safety_node] -> /safe_cmd_vel -> motor_controller.

Rôle : watchdog + validation + protection finale (PAS un doublon de Collision Monitor).
  - arrêt d'urgence logiciel verrouillé (/emergency_stop, Bool)
  - timeout /cmd_vel, timeout /scan (données critiques absentes -> commande ZÉRO)
  - rejet NaN/Inf, limitation de vitesse
  - hard-stop de dernier recours : quelques points LiDAR juste devant (marche avant)
    ou derrière (marche arrière) le châssis. Une rotation pure n'est jamais bloquée.
  - publie /safe_cmd_vel en continu (heartbeat) et /safety_state (String)

Aucun sleep bloquant. Les blocages sont loggués (throttle) sauf les états de démarrage.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('monitor_scan', True)
        self.declare_parameter('scan_timeout', 1.0)
        self.declare_parameter('hard_stop_enabled', True)
        self.declare_parameter('footprint_half_length', 0.285)
        self.declare_parameter('footprint_half_width', 0.20)
        self.declare_parameter('laser_x', 0.0)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('hard_stop_front_margin', 0.06)
        self.declare_parameter('hard_stop_rear_margin', 0.10)
        self.declare_parameter('hard_stop_lateral_margin', 0.02)
        self.declare_parameter('hard_stop_min_points', 3)
        self.declare_parameter('min_valid_range', 0.15)
        self.declare_parameter('warn_period', 5.0)

        g = self.get_parameter
        self.cmd_timeout = float(g('cmd_timeout').value)
        self.max_lin = float(g('max_linear_speed').value)
        self.max_ang = float(g('max_angular_speed').value)
        self.monitor_scan = bool(g('monitor_scan').value)
        self.scan_timeout = float(g('scan_timeout').value)
        self.hard_stop = bool(g('hard_stop_enabled').value)
        self.half_len = float(g('footprint_half_length').value)
        self.half_wid = float(g('footprint_half_width').value)
        self.laser_x = float(g('laser_x').value)
        self.laser_y = float(g('laser_y').value)
        self.front_margin = float(g('hard_stop_front_margin').value)
        self.rear_margin = float(g('hard_stop_rear_margin').value)
        self.lat_margin = float(g('hard_stop_lateral_margin').value)
        self.min_points = int(g('hard_stop_min_points').value)
        self.min_valid_range = float(g('min_valid_range').value)
        self.warn_period = float(g('warn_period').value)

        self._last_cmd = Twist()
        self._last_cmd_time = None
        self._last_scan_time = None
        self._estop = False
        self._front_count = 0
        self._rear_count = 0
        self._last_warn_time = -1e9
        self._last_state = None
        self._last_state_pub = -1e9

        self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Bool, 'emergency_stop', self._estop_cb, 10)
        # /scan est publié en best-effort : QoS SensorData obligatoire.
        if self.monitor_scan:
            self.create_subscription(LaserScan, 'scan', self._scan_cb, qos_profile_sensor_data)

        self.pub = self.create_publisher(Twist, 'safe_cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'safety_state', 10)
        self.create_timer(1.0 / float(g('publish_rate').value), self._tick)

        self.get_logger().info('Safety node actif : /cmd_vel -> /safe_cmd_vel')

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _warn_throttled(self, text):
        now = self._now_sec()
        if now - self._last_warn_time >= self.warn_period:
            self._last_warn_time = now
            self.get_logger().warn(text)

    # ---------- callbacks ----------
    def _cmd_cb(self, msg: Twist):
        vals = (msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(v) for v in vals):
            self.get_logger().error('Commande /cmd_vel invalide (NaN/Inf) ignorée.')
            self._last_cmd = Twist()          # on ne garde pas l'ancienne commande
            self._last_cmd_time = None
            return
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _scan_cb(self, msg: LaserScan):
        self._last_scan_time = self.get_clock().now()
        if not self.hard_stop:
            return
        front = rear = 0
        y_lim = self.half_wid + self.lat_margin
        f_hi = self.half_len + self.front_margin
        r_hi = self.half_len + self.rear_margin
        rmin = max(msg.range_min, self.min_valid_range)
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and rmin <= r <= msg.range_max:
                x = self.laser_x + r * math.cos(angle)
                y = self.laser_y + r * math.sin(angle)
                if abs(y) <= y_lim:
                    if self.half_len < x <= f_hi:
                        front += 1
                    elif -r_hi <= x < -self.half_len:
                        rear += 1
            angle += msg.angle_increment
        self._front_count = front
        self._rear_count = rear

    def _estop_cb(self, msg: Bool):
        if msg.data and not self._estop:
            self.get_logger().error("ARRÊT D'URGENCE ACTIVÉ — moteurs STOP.")
        if not msg.data and self._estop:
            self.get_logger().warn("Arrêt d'urgence réarmé.")
        self._estop = msg.data

    # ---------- logique ----------
    def _blocked_reason(self, v_out):
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
        if self.hard_stop:
            if v_out > 0.005 and self._front_count >= self.min_points:
                return 'HARD_STOP_FRONT'
            if v_out < -0.005 and self._rear_count >= self.min_points:
                return 'HARD_STOP_REAR'
        return None

    def _tick(self):
        v = max(-self.max_lin, min(self.max_lin, self._last_cmd.linear.x))
        w = max(-self.max_ang, min(self.max_ang, self._last_cmd.angular.z))
        reason = self._blocked_reason(v)

        out = Twist()
        if reason is None:
            out.linear.x = v
            out.angular.z = w
        elif reason not in ('NO_CMD_YET', 'NO_SCAN_YET', 'CMD_TIMEOUT'):
            self._warn_throttled(f'Blocage sécurité : {reason}')
        self.pub.publish(out)

        state = reason or 'OK'
        now = self._now_sec()
        if state != self._last_state or now - self._last_state_pub > 1.0:
            self._last_state, self._last_state_pub = state, now
            self.state_pub.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())  # STOP final
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

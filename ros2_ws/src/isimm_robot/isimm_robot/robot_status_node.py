"""
Robot Status Node — ISIMM ROBOT.

Surveille les flux vitaux et publie un diagnostic 1 Hz sur /robot_status
(diagnostic_msgs/DiagnosticArray). Utile pour le dépannage et un futur
tableau de bord. Ne loggue que les CHANGEMENTS d'état.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class RobotStatusNode(Node):
    def __init__(self):
        super().__init__('robot_status_node')

        self.declare_parameter('scan_timeout', 2.0)
        self.declare_parameter('imu_timeout', 2.0)

        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.imu_timeout = float(self.get_parameter('imu_timeout').value)

        self._last_scan = None
        self._last_imu = None
        self._last_safe_cmd = None

        self.create_subscription(LaserScan, 'scan', lambda m: self._set('_last_scan'), 10)
        self.create_subscription(Imu, 'imu', lambda m: self._set('_last_imu'), 10)
        self.create_subscription(Twist, 'safe_cmd_vel', lambda m: self._set('_last_safe_cmd'), 10)

        self.pub = self.create_publisher(DiagnosticArray, 'robot_status', 10)
        self.create_timer(1.0, self._tick)
        self._prev_states = {}

    def _set(self, attr):
        setattr(self, attr, self.get_clock().now())

    @staticmethod
    def _age(now, t):
        return float('inf') if t is None else (now - t).nanoseconds * 1e-9

    def _check(self, name, age, timeout):
        ok = age <= timeout
        state = 'OK' if ok else 'STALE'
        if self._prev_states.get(name) != state:
            self._prev_states[name] = state
            log = self.get_logger().info if ok else self.get_logger().error
            log(f'{name} : {"OK" if ok else f"ABSENT depuis {age:.1f}s"}')
        return KeyValue(key=name, value=f'{"OK" if ok else "STALE"} ({age:.1f}s)')

    def _tick(self):
        now = self.get_clock().now()
        arr = DiagnosticArray()
        arr.header.stamp = now.to_msg()
        st = DiagnosticStatus()
        st.name = 'isimm_robot'
        st.values.append(self._check('lidar_scan', self._age(now, self._last_scan), self.scan_timeout))
        st.values.append(self._check('imu', self._age(now, self._last_imu), self.imu_timeout))
        st.values.append(self._check('safe_cmd_vel', self._age(now, self._last_safe_cmd), 2.0))
        st.level = DiagnosticStatus.OK if all('OK' in v.value for v in st.values) else DiagnosticStatus.ERROR
        st.message = 'système nominal' if st.level == DiagnosticStatus.OK else 'un ou plusieurs flux absents'
        arr.status.append(st)
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

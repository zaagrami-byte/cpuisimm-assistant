"""
Robot Status Node — ISIMM ROBOT.

Diagnostic 1 Hz sur /robot_status (DiagnosticArray), un status par composant + 'overall'.
Ne loggue que les CHANGEMENTS de niveau.

  ERROR : LiDAR, /odom (RF2O), TF odom->base_link, safety_node, motor/Arduino,
          (mapping/navigation) TF map->odom + slam_toolbox,
          (navigation) nœuds Nav2 non 'active'
  WARN  : IMU (non fusionnée dans l'odométrie : n'empêche PAS la navigation)
Paramètre `mode` : base | mapping | navigation.
"""

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

LEVELS = [DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR]
NAV2_NODES = ['controller_server', 'planner_server', 'behavior_server',
              'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower']


class RobotStatusNode(Node):
    def __init__(self):
        super().__init__('robot_status_node')
        d = self.declare_parameter
        d('mode', 'base')
        d('scan_timeout', 2.0)
        d('odom_timeout', 2.0)
        d('imu_timeout', 2.0)
        d('tf_timeout', 2.0)
        d('safe_cmd_timeout', 1.0)
        d('motor_status_timeout', 3.0)
        g = self.get_parameter
        self.mode = str(g('mode').value)
        self.to = {k: float(g(k).value) for k in
                   ('scan_timeout', 'odom_timeout', 'imu_timeout', 'tf_timeout',
                    'safe_cmd_timeout', 'motor_status_timeout')}

        self._t = {}
        self._motor_status = None
        self._safety_state = None

        self.create_subscription(LaserScan, 'scan', lambda m: self._stamp('scan'), qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', lambda m: self._stamp('odom'), 10)
        self.create_subscription(Imu, 'imu', lambda m: self._stamp('imu'), 10)
        self.create_subscription(Twist, 'safe_cmd_vel', lambda m: self._stamp('safe'), 10)
        self.create_subscription(String, 'motor_status', self._motor_cb, 10)
        self.create_subscription(String, 'safety_state', self._safety_cb, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._lc_names = []
        if self.mode in ('mapping', 'navigation'):
            self._lc_names.append('slam_toolbox')
        if self.mode == 'navigation':
            self._lc_names += NAV2_NODES
        self._lc_clients = {n: self.create_client(GetState, f'/{n}/get_state') for n in self._lc_names}
        self._lc_futures = {n: None for n in self._lc_names}
        self._lc_state = {n: None for n in self._lc_names}

        self.pub = self.create_publisher(DiagnosticArray, 'robot_status', 10)
        self._prev = {}
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f'Status node : mode={self.mode}')

    # ---------- callbacks ----------
    def _stamp(self, key):
        self._t[key] = self.get_clock().now()

    def _motor_cb(self, msg):
        self._motor_status = msg.data
        self._stamp('motor')

    def _safety_cb(self, msg):
        self._safety_state = msg.data

    # ---------- helpers ----------
    def _age(self, key, now):
        t = self._t.get(key)
        return None if t is None else (now - t).nanoseconds * 1e-9

    def _tf_age(self, parent, child, now):
        try:
            tr = self._tf_buffer.lookup_transform(parent, child, Time())
            return (now - Time.from_msg(tr.header.stamp)).nanoseconds * 1e-9
        except TransformException:
            return None

    def _flow(self, name, key, timeout, bad_level, now):
        age = self._age(key, now)
        if age is None:
            return (name, bad_level, 'absent')
        if age > timeout:
            return (name, bad_level, f'STALE depuis {age:.1f}s')
        return (name, 0, f'OK ({age:.1f}s)')

    def _tf(self, name, parent, child, level, now):
        age = self._tf_age(parent, child, now)
        if age is None:
            return (name, level, f'{parent}->{child} indisponible')
        if age > self.to['tf_timeout']:
            return (name, level, f'{parent}->{child} périmé ({age:.1f}s)')
        return (name, 0, f'OK ({age:.1f}s)')

    def _poll_lifecycle(self):
        for n, client in self._lc_clients.items():
            fut = self._lc_futures[n]
            if fut is not None and fut.done():
                try:
                    self._lc_state[n] = fut.result().current_state.label
                except Exception:
                    self._lc_state[n] = 'error'
                self._lc_futures[n] = fut = None
            if fut is None:
                if client.service_is_ready():
                    self._lc_futures[n] = client.call_async(GetState.Request())
                else:
                    self._lc_state[n] = None

    # ---------- diagnostic ----------
    def _tick(self):
        now = self.get_clock().now()
        self._poll_lifecycle()
        items = [
            self._flow('lidar_scan', 'scan', self.to['scan_timeout'], 2, now),
            self._flow('odom_rf2o', 'odom', self.to['odom_timeout'], 2, now),
            self._tf('tf_odom_base_link', 'odom', 'base_link', 2, now),
            self._flow('imu', 'imu', self.to['imu_timeout'], 1, now),
            self._flow('safety_node', 'safe', self.to['safe_cmd_timeout'], 2, now),
        ]
        if self._safety_state not in (None, 'OK'):
            items.append(('safety_state', 1, f'blocage : {self._safety_state}'))
        else:
            items.append(('safety_state', 0, self._safety_state or 'inconnu'))

        m_age = self._age('motor', now)
        if m_age is None or m_age > self.to['motor_status_timeout']:
            items.append(('motor_controller', 2, 'absent'))
        elif self._motor_status == 'OK':
            items.append(('motor_controller', 0, 'OK (liaison série + Arduino)'))
        else:
            items.append(('motor_controller', 2, str(self._motor_status)))

        if self.mode in ('mapping', 'navigation'):
            items.append(self._tf('tf_map_odom', 'map', 'odom', 2, now))
            for n in self._lc_names:
                st = self._lc_state[n]
                if st == 'active':
                    items.append((n, 0, 'active'))
                else:
                    items.append((n, 2, f'état: {st if st else "absent"}'))

        arr = DiagnosticArray()
        arr.header.stamp = now.to_msg()
        worst = 0
        for name, lvl, msg in items:
            worst = max(worst, lvl)
            st = DiagnosticStatus()
            st.name = f'isimm/{name}'
            st.hardware_id = 'isimm_robot'
            st.level = LEVELS[lvl]
            st.message = msg
            st.values.append(KeyValue(key='detail', value=msg))
            arr.status.append(st)
            if self._prev.get(name) != lvl:
                self._prev[name] = lvl
                text = f'{name} : {msg}'
                if lvl == 0:
                    self.get_logger().info(text)
                elif lvl == 1:
                    self.get_logger().warn(text)
                else:
                    self.get_logger().error(text)
        overall = DiagnosticStatus()
        overall.name = 'isimm/overall'
        overall.hardware_id = 'isimm_robot'
        overall.level = LEVELS[worst]
        overall.message = ('système nominal', 'dégradé (non bloquant)', 'composant critique en défaut')[worst]
        arr.status.insert(0, overall)
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
        if rclpy.ok():
            rclpy.shutdown()

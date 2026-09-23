"""
ESP32 Sensor Node — ISIMM ROBOT.

Lit le port série USB de l'ESP32 et publie :
  /imu   (sensor_msgs/Imu)      <- lignes "I,ax,ay,az,gx,gy,gz,temp"
  /range/front_left|front_right|rear_left|rear_right (sensor_msgs/Range)
                                 <- lignes "U,fl,fr,rl,rr" (ultrasons futurs)

Honnêteté capteur : MPU6050 brut sans fusion -> orientation non estimée
(covariance[0] = -1). Les vitesses angulaires et accélérations sont valides.
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float64

import serial

RANGE_FIELDS = ['front_left', 'front_right', 'rear_left', 'rear_right']


class Esp32SensorNode(Node):
    def __init__(self):
        super().__init__('esp32_sensor_node')

        self.declare_parameter('serial_port', '/dev/isimm_esp32')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('range_frame_id', 'ultrasonic_link')
        self.declare_parameter('reconnect_period', 5.0)

        self.port = self.get_parameter('serial_port').value
        self.baud = int(self.get_parameter('serial_baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.range_frame_id = self.get_parameter('range_frame_id').value

        sensor_qos = QoSProfile(depth=10)
        self.imu_pub = self.create_publisher(Imu, 'imu', sensor_qos)
        self.range_pubs = {
            name: self.create_publisher(Range, f'range/{name}', sensor_qos)
            for name in RANGE_FIELDS
        }

        self._stop = False
        self._ser = None
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info(f'ESP32 node démarré, attente de {self.port}')

    # ---------- thread de lecture ----------
    def _open(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.5)
            self.get_logger().info(f'ESP32 connectée : {self.port}')
        except serial.SerialException as e:
            self.get_logger().warn(f'ESP32 non disponible ({self.port}) : {e}')
            self._ser = None

    def _reader_loop(self):
        while rclpy.ok() and not self._stop:
            if self._ser is None:
                self._open()
                if self._ser is None:
                    self._wait(5.0)
                    continue
            try:
                line = self._ser.readline().decode('ascii', errors='replace').strip()
            except serial.SerialException:
                self.get_logger().error('Perte du lien ESP32, reconnexion...')
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                continue
            if not line:
                continue
            try:
                self._handle_line(line)
            except ValueError:
                self.get_logger().warn(f'Trame invalide ignorée : {line}')

    def _wait(self, seconds):
        import time
        time.sleep(seconds)

    # ---------- traitement ----------
    def _handle_line(self, line: str):
        parts = line.split(',')
        tag = parts[0]

        if tag == 'I' and len(parts) == 8:
            ax, ay, az, gx, gy, gz = (float(x) for x in parts[1:7])
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            # pas d'orientation estimée : on le déclare honnêtement
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            for i in range(3):
                msg.angular_velocity_covariance[i * 4] = 0.01
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            for i in range(3):
                msg.linear_acceleration_covariance[i * 4] = 0.1
            self.imu_pub.publish(msg)

        elif tag == 'U' and len(parts) == 5:
            # ultrasons futurs : "U,<fl>,<fr>,<rl>,<rr>" en mètres
            for name, value in zip(RANGE_FIELDS, parts[1:]):
                r = Range()
                r.header.stamp = self.get_clock().now().to_msg()
                r.header.frame_id = self.range_frame_id
                r.radiation_type = Range.ULTRASOUND
                r.field_of_view = 0.26
                r.min_range = 0.02
                r.max_range = 4.0
                r.range = float(value)
                self.range_pubs[name].publish(r)

        elif tag == 'E':
            self.get_logger().error(f'ESP32 signale une erreur : {line}')
        # "READY", "OK" -> ignorés silencieusement

    def destroy_node(self):
        self._stop = True
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

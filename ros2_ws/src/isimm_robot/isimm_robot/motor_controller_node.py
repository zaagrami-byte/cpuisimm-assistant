"""
Motor Controller Node — ISIMM ROBOT.

Entrée  : /safe_cmd_vel (geometry_msgs/Twist)   [sortie du safety_node]
Sortie  : port série USB vers Arduino UNO.

Conversion différentielle :
    v_left  = v - omega * wheel_separation / 2
    v_right = v + omega * wheel_separation / 2
    pwm = |v_wheel| / (v_max + w_max*L/2) * max_pwm   (deadband + min_pwm)

Envoie les commandes à command_rate Hz, y compris des commandes nulles :
cela sert de heartbeat pour le watchdog Arduino. Si le port série tombe,
les envois cessent -> l'Arduino freine seul après CMD_TIMEOUT_MS.

Sécurité additionnelle : timeout local cmd_timeout sans /safe_cmd_vel -> BRAKE.

Reconnexion : UN SEUL timer périodique gère la réouverture du port
(pas de création de timers en cascade en cas d'erreur répétée).
"""

import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')

        # ---- paramètres ----
        self.declare_parameter('serial_port', '/dev/isimm_arduino')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('wheel_separation', 0.48)    # validé (m)
        self.declare_parameter('max_linear_speed', 0.5)     # m/s
        self.declare_parameter('max_angular_speed', 1.5)    # rad/s
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('min_pwm', 25)               # sous ce PWM le moteur ne tourne pas
        self.declare_parameter('deadband', 0.01)            # m/s sous lequel on considère 0
        self.declare_parameter('command_rate', 20.0)        # Hz (heartbeat)
        self.declare_parameter('cmd_timeout', 0.5)          # s sans /safe_cmd_vel -> BRAKE local
        self.declare_parameter('retry_period', 5.0)         # s entre deux tentatives de reconnexion
        self.declare_parameter('left_invert', False)
        self.declare_parameter('right_invert', False)

        self.port = self.get_parameter('serial_port').value
        self.baud = int(self.get_parameter('serial_baudrate').value)
        self.wheel_sep = float(self.get_parameter('wheel_separation').value)
        self.max_lin = float(self.get_parameter('max_linear_speed').value)
        self.max_ang = float(self.get_parameter('max_angular_speed').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.min_pwm = int(self.get_parameter('min_pwm').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.left_invert = bool(self.get_parameter('left_invert').value)
        self.right_invert = bool(self.get_parameter('right_invert').value)

        self.v_wheel_max = self.max_lin + self.max_ang * self.wheel_sep / 2.0

        # état
        self._lock = threading.Lock()
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._last_cmd_time = None
        self._ser = None

        self._open_serial()

        self.create_subscription(Twist, 'safe_cmd_vel', self._cmd_cb, 10)
        period = 1.0 / float(self.get_parameter('command_rate').value)
        self.create_timer(period, self._send_timer)

        # Timer de reconnexion UNIQUE, créé une seule fois. Il ne fait rien
        # tant que le port est ouvert (aucune fuite de timers en cas d'échecs répétés).
        self.create_timer(
            float(self.get_parameter('retry_period').value), self._retry_serial)

        self.get_logger().info(
            f'Motor controller prêt : {self.port}@{self.baud}, '
            f'L={self.wheel_sep} m, v_max={self.max_lin} m/s, '
            f'cmd_timeout={self.cmd_timeout} s')

    # ---------- série ----------
    def _open_serial(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f'Port série ouvert : {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(
                f"Impossible d'ouvrir {self.port} : {e}. "
                "Vérifiez le câble, les permissions (dialout) et l'udev. "
                "Nouvelle tentative périodique en arrière-plan.")
            self._ser = None

    def _retry_serial(self):
        """Appelé périodiquement : ne fait rien si le port est déjà ouvert."""
        if self._ser is not None and self._ser.is_open:
            return
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f'Port série rouvert : {self.port}')
        except serial.SerialException:
            pass  # le timer réessaiera au prochain cycle

    # ---------- commande ----------
    def _cmd_cb(self, msg: Twist):
        if not (math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)):
            self.get_logger().error('Commande /safe_cmd_vel invalide (NaN/Inf) ignorée.')
            return
        with self._lock:
            self._cmd_v = msg.linear.x
            self._cmd_w = msg.angular.z
            self._last_cmd_time = self.get_clock().now().nanoseconds * 1e-9

    def _wheel_to_pwm(self, v_wheel: float) -> int:
        """Vitesse linéaire d'une roue -> PWM avec deadband et min_pwm."""
        mag = abs(v_wheel)
        if mag < self.deadband:
            return 0
        pwm = int(round(mag / self.v_wheel_max * self.max_pwm))
        pwm = max(self.min_pwm, min(self.max_pwm, pwm))  # deadband moteur
        return pwm

    @staticmethod
    def _dir(v_wheel: float, invert: bool) -> int:
        forward = v_wheel >= 0.0
        if invert:
            forward = not forward
        return 1 if forward else 0

    def _send_timer(self):
        if self._ser is None or not self._ser.is_open:
            return  # la reconnexion est gérée par _retry_serial
        with self._lock:
            v, w = self._cmd_v, self._cmd_w
            last_cmd_time = self._last_cmd_time

        # Sécurité locale indépendante du watchdog Arduino.
        now = self.get_clock().now().nanoseconds * 1e-9
        if last_cmd_time is None or (now - last_cmd_time) > self.cmd_timeout:
            v, w = 0.0, 0.0

        # clamp
        v = max(-self.max_lin, min(self.max_lin, v))
        w = max(-self.max_ang, min(self.max_ang, w))

        v_left = v - w * self.wheel_sep / 2.0
        v_right = v + w * self.wheel_sep / 2.0

        lpwm = self._wheel_to_pwm(v_left)
        rpwm = self._wheel_to_pwm(v_right)
        ldir = self._dir(v_left, self.left_invert)
        rdir = self._dir(v_right, self.right_invert)
        lbrk = 1 if lpwm == 0 else 0
        rbrk = 1 if rpwm == 0 else 0

        frame = f'M,{lpwm},{ldir},{lbrk},{rpwm},{rdir},{rbrk}\n'
        try:
            self._ser.write(frame.encode('ascii'))
            self._ser.flush()
        except serial.SerialException as e:
            self.get_logger().error(f'Écriture série échouée : {e}')
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None  # _retry_serial prendra le relais

    def _stop_and_close(self):
        if self._ser is not None and self._ser.is_open:
            try:
                self._ser.write(b'M,0,1,1,0,1,1\n')  # BRAKE les deux moteurs
                self._ser.flush()
                self._ser.close()
            except Exception:
                pass

    def destroy_node(self):
        self._stop_and_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

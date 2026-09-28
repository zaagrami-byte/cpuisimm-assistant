"""
Motor Controller Node — ISIMM ROBOT.

Entrée  : /safe_cmd_vel (Twist)  [sortie du safety_node]
Sortie  : port série vers Arduino UNO, trame  M,<lpwm>,<ldir>,<lbrk>,<rpwm>,<rdir>,<rbrk>

Sans encodeurs, la commande est BOUCLE OUVERTE (Nav2 ferme la boucle via RF2O/localisation).
  v_left  = v - w*L/2 ; v_right = v + w*L/2
  gains gauche/droite (calibration statique) puis désaturation qui conserve le rapport G/D
  |v| < deadband                -> PWM 0 + frein (vrai zéro, jamais de min_pwm)
  deadband <= |v| <= v_min_eff  -> min_pwm
  v_min_eff < |v| <= v_at_max   -> interpolation linéaire min_pwm .. max_pwm
  PWM jamais > max_pwm (et jamais > 255).

Heartbeat : trames à command_rate Hz, y compris zéro. Si le Pi/ROS/USB tombe, l'Arduino
freine seul après CMD_TIMEOUT_MS. Timeout local cmd_timeout sans /safe_cmd_vel -> BRAKE.
Lit les réponses Arduino (OK/READY/ERR:*/ESTOP:*), ping 1 Hz, publie /motor_status.
E-stop : /emergency_stop True -> 'S' ; False -> 'C'. Service ~/clear_estop -> 'C'
(bouton physique). Un seul timer de reconnexion.
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

import serial

HARD_MAX_PWM = 255


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')

        d = self.declare_parameter
        d('serial_port', '/dev/isimm_arduino')
        d('serial_baudrate', 115200)
        d('wheel_separation', 0.48)
        d('max_linear_speed', 0.35)
        d('max_angular_speed', 1.2)
        d('max_pwm', 100)
        d('left_min_pwm', 25)
        d('right_min_pwm', 25)
        d('wheel_speed_min_effective', 0.05)
        d('wheel_speed_at_max_pwm', 0.64)
        d('left_gain', 1.0)
        d('right_gain', 1.0)
        d('deadband', 0.02)
        d('command_rate', 20.0)
        d('cmd_timeout', 0.5)
        d('retry_period', 5.0)
        d('startup_grace', 2.0)
        d('left_invert', False)
        d('right_invert', False)

        g = self.get_parameter
        self.port = g('serial_port').value
        self.baud = int(g('serial_baudrate').value)
        self.wheel_sep = float(g('wheel_separation').value)
        self.max_lin = float(g('max_linear_speed').value)
        self.max_ang = float(g('max_angular_speed').value)
        self.max_pwm = max(1, min(HARD_MAX_PWM, int(g('max_pwm').value)))
        self.lmin = max(0, min(self.max_pwm - 1, int(g('left_min_pwm').value)))
        self.rmin = max(0, min(self.max_pwm - 1, int(g('right_min_pwm').value)))
        self.v_min = float(g('wheel_speed_min_effective').value)
        self.v_max = float(g('wheel_speed_at_max_pwm').value)
        self.deadband = float(g('deadband').value)
        self.cmd_timeout = float(g('cmd_timeout').value)
        self.grace = float(g('startup_grace').value)
        self.left_invert = bool(g('left_invert').value)
        self.right_invert = bool(g('right_invert').value)

        if not (0.0 <= self.deadband < self.v_min < self.v_max):
            raise ValueError('Il faut 0 <= deadband < wheel_speed_min_effective < wheel_speed_at_max_pwm')

        self.lgain = self._clamp_gain('left_gain')
        self.rgain = self._clamp_gain('right_gain')

        self._lock = threading.Lock()
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._last_cmd_time = None
        self._ser = None
        self._opened_at = 0.0
        self._rx = b''
        self._last_ok = 0.0
        self._last_ping = 0.0
        self._arduino_estop = False
        self._estop_sw = False

        self._open_serial()

        self.create_subscription(Twist, 'safe_cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Bool, 'emergency_stop', self._estop_cb, 10)
        self.create_service(Trigger, '~/clear_estop', self._clear_estop_srv)
        self.status_pub = self.create_publisher(String, 'motor_status', 10)

        self.create_timer(1.0 / float(g('command_rate').value), self._send_timer)
        self.create_timer(float(g('retry_period').value), self._retry_serial)
        self.create_timer(1.0, self._status_timer)

        self.get_logger().info(
            f'Motor controller prêt : {self.port}@{self.baud}, L={self.wheel_sep} m, '
            f'max_pwm={self.max_pwm}, min_pwm=({self.lmin},{self.rmin}), '
            f'gains=({self.lgain:.3f},{self.rgain:.3f}), cmd_timeout={self.cmd_timeout} s')

    def _clamp_gain(self, name):
        val = float(self.get_parameter(name).value)
        clamped = max(0.5, min(1.5, val))
        if clamped != val:
            self.get_logger().warn(f'{name}={val} hors [0.5, 1.5] -> {clamped}')
        return clamped

    # ---------- série ----------
    def _open_serial(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0, write_timeout=0.1)
            self._opened_at = time.monotonic()
            self._rx = b''
            self._last_ok = 0.0
            self.get_logger().info(f'Port série ouvert : {self.port} (reset Uno ~{self.grace}s)')
            return True
        except serial.SerialException as e:
            self.get_logger().error(
                f"Impossible d'ouvrir {self.port} : {e}. Vérifiez câble, groupe dialout, udev.")
            self._ser = None
            return False

    def _retry_serial(self):
        if not self._ready():
            self._open_serial()

    def _ready(self):
        return self._ser is not None and self._ser.is_open

    def _drop_serial(self):
        try:
            self._ser.close()
        except Exception:
            pass
        self._ser = None

    def _write(self, text: str) -> bool:
        if not self._ready():
            return False
        try:
            self._ser.write(text.encode('ascii'))
            self._ser.flush()
            return True
        except serial.SerialException as e:   # inclut SerialTimeoutException
            self.get_logger().error(f'Écriture série échouée : {e}')
            self._drop_serial()
            return False

    def _poll_arduino(self):
        try:
            n = self._ser.in_waiting
            if n:
                self._rx += self._ser.read(n)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Lecture série échouée : {e}')
            self._drop_serial()
            return
        while b'\n' in self._rx:
            raw, self._rx = self._rx.split(b'\n', 1)
            line = raw.decode('ascii', errors='replace').strip()
            if not line:
                continue
            if line == 'OK':
                self._last_ok = time.monotonic()
            elif line == 'READY':
                self.get_logger().info('Arduino : READY')
            elif line.startswith('ESTOP:ON') or line == 'ERR:ESTOP':
                if not self._arduino_estop:
                    self.get_logger().error('Arduino en E-STOP (envoyer clear_estop pour réarmer).')
                self._arduino_estop = True
            elif line == 'ESTOP:OFF':
                self._arduino_estop = False
                self.get_logger().info('Arduino : E-stop réarmé.')
            elif line.startswith('ERR'):
                self.get_logger().warn(f'Arduino : {line}', throttle_duration_sec=2.0)

    # ---------- commande ----------
    def _cmd_cb(self, msg: Twist):
        if not (math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)):
            self.get_logger().error('Commande /safe_cmd_vel invalide (NaN/Inf) ignorée.')
            return
        with self._lock:
            self._cmd_v = msg.linear.x
            self._cmd_w = msg.angular.z
            self._last_cmd_time = time.monotonic()

    def _estop_cb(self, msg: Bool):
        if msg.data and not self._estop_sw:
            self._estop_sw = True
            self._write('S\n')
        elif not msg.data and self._estop_sw:
            self._estop_sw = False
            self._write('C\n')

    def _clear_estop_srv(self, request, response):
        ok = self._write('C\n')
        response.success = ok
        response.message = 'C envoyé' if ok else 'port série indisponible'
        return response

    def _wheel_to_pwm(self, v_eff: float, min_pwm: int) -> int:
        mag = abs(v_eff)
        if mag < self.deadband:
            return 0
        if mag <= self.v_min:
            pwm = min_pwm
        else:
            frac = (mag - self.v_min) / (self.v_max - self.v_min)
            pwm = min_pwm + frac * (self.max_pwm - min_pwm)
        return int(round(max(min_pwm, min(self.max_pwm, pwm))))

    @staticmethod
    def _dir(v_wheel: float, invert: bool) -> int:
        forward = v_wheel >= 0.0
        if invert:
            forward = not forward
        return 1 if forward else 0

    def build_frame(self, v: float, w: float) -> str:
        v = max(-self.max_lin, min(self.max_lin, v))
        w = max(-self.max_ang, min(self.max_ang, w))
        vl = v - w * self.wheel_sep / 2.0
        vr = v + w * self.wheel_sep / 2.0
        vl_e, vr_e = vl * self.lgain, vr * self.rgain
        peak = max(abs(vl_e), abs(vr_e))
        if peak > self.v_max:                    # désaturation : garde la courbure
            s = self.v_max / peak
            vl_e, vr_e = vl_e * s, vr_e * s
        lpwm = self._wheel_to_pwm(vl_e, self.lmin)
        rpwm = self._wheel_to_pwm(vr_e, self.rmin)
        ldir = self._dir(vl, self.left_invert)
        rdir = self._dir(vr, self.right_invert)
        lbrk = 1 if lpwm == 0 else 0
        rbrk = 1 if rpwm == 0 else 0
        return f'M,{lpwm},{ldir},{lbrk},{rpwm},{rdir},{rbrk}\n'

    def _send_timer(self):
        if not self._ready():
            return
        self._poll_arduino()
        if not self._ready():
            return
        now = time.monotonic()
        if now - self._opened_at < self.grace:
            return
        with self._lock:
            v, w, t = self._cmd_v, self._cmd_w, self._last_cmd_time
        if t is None or (now - t) > self.cmd_timeout or self._estop_sw:
            v, w = 0.0, 0.0
        if not self._write(self.build_frame(v, w)):
            return
        if now - self._last_ping >= 1.0:
            self._last_ping = now
            self._write('P\n')

    def _status_timer(self):
        now = time.monotonic()
        if not self._ready():
            s = 'NO_SERIAL'
        elif now - self._opened_at < self.grace + 3.0 and self._last_ok == 0.0:
            s = 'STARTING'
        elif now - self._last_ok > 3.0:
            s = 'NO_ARDUINO_REPLY'
        elif self._arduino_estop:
            s = 'ARDUINO_ESTOP'
        else:
            s = 'OK'
        self.status_pub.publish(String(data=s))

    def _stop_and_close(self):
        if self._ready():
            try:
                self._ser.write(b'M,0,1,1,0,1,1\n')   # BRAKE
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
        if rclpy.ok():
            rclpy.shutdown()

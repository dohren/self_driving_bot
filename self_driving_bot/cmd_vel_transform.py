#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

# --- kurzer Gegendreh-Impuls + Hysterese-Einstellungen ---
COUNTER_W   = 0.4     # geringere Gegenkraft
COUNTER_MS  = 150     # kürzere Dauer
COUNTER_HZ  = 20.0

EPS_W_ZERO  = 0.12    # höher setzen: Loslassen sicherer erkennen
TURN_ON     = 0.1    # Hysterese: ab hier sicher TURN
TURN_OFF    = 0.8    # Hysterese: darunter TURN beenden
JOY_NOISE_V = 0.05    # Deadband linear
JOY_NOISE_W = 0.05    # Deadband angular

class CmdVelTransformer(Node):
    def __init__(self):
        super().__init__('cmd_vel_transformer')
        self.declare_parameter('ticks_meter', 3400.0)
        self.ticks_meter = float(self.get_parameter('ticks_meter').value)

        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.v_actual = 0.0
        self.prev_time = self.get_clock().now()

        self.prev_w = 0.0
        self._pulse_timer = None
        self._pulses_left = 0
        self._pulse_sign = 0.0

        # Modusspeicher für Hysterese
        self._mode = 'IDLE'  # 'IDLE' | 'DRIVE' | 'TURN'

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int32, '/left_ticks', self.left_callback, 10)
        self.create_subscription(Int32, '/right_ticks', self.right_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_transform', 10)
        self.get_logger().info('CmdVelTransformer (exklusiv + Hysterese) aktiv')

    def left_callback(self, msg: Int32):
        self.left_ticks = msg.data
        self.update_velocity()

    def right_callback(self, msg: Int32):
        self.right_ticks = msg.data
        self.update_velocity()

    def update_velocity(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt == 0:
            return
        if self.prev_left_ticks is not None and self.prev_right_ticks is not None:
            delta_left = self.left_ticks - self.prev_left_ticks
            delta_right = self.right_ticks - self.prev_right_ticks
            delta_distance = (delta_left + delta_right) / 2.0
            self.v_actual = (delta_distance / self.ticks_meter) / dt
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks
        self.prev_time = now

    def _decide_mode(self, v: float, w: float) -> str:
        # Hysterese: nur wechseln, wenn wir klare Schwellen überschreiten
        if abs(w) >= TURN_ON and abs(w) >= abs(v):
            self._mode = 'TURN'
        elif abs(v) > abs(w):
            self._mode = 'DRIVE'
        elif abs(w) <= TURN_OFF and abs(v) < JOY_NOISE_V:
            self._mode = 'IDLE'
        # sonst: behalte bisherigen Modus
        return self._mode

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        mode = self._decide_mode(v, w)
        self.get_logger().info(f"MODE: {mode} (|v|={abs(v):.3f}, |w|={abs(w):.3f})")

        # Gegenimpuls starten, wenn zuvor gedreht und jetzt losgelassen
        started_pulse = False
        if (abs(self.prev_w) > 0.2) and (abs(w) < EPS_W_ZERO) and (abs(v) < JOY_NOISE_V):
            sign = -1.0 if self.prev_w > 0 else 1.0
            self._start_counter_pulse(sign)
            started_pulse = True

        # Gegenimpuls abbrechen, sobald wieder echter Befehl kommt
        if (abs(w) >= EPS_W_ZERO) or (abs(v) >= JOY_NOISE_V):
            if self._pulse_timer is not None:
                self._pulse_timer.cancel()
                self._pulse_timer = None
                self._pulses_left = 0

        # Während des gerade gestarteten Impulses sonst nichts senden
        if started_pulse and abs(w) < EPS_W_ZERO and abs(v) < JOY_NOISE_V:
            self.prev_w = w
            return

        # Korrektur-Parameter
        linear_scale = 0.5
        angular_scale = 0.25
        min_linear = 0.55
        min_angular = 0.7
        MAX_LINEAR = 0.65
        MAX_ANGULAR = 0.70  # wie gewünscht gesenkt

        v_corrected = 0.0
        w_corrected = 0.0

        if mode == 'DRIVE':
            # Nur Vorwärtsfahrt (Drehen bleibt 0)
            v_corrected = v * linear_scale
            if abs(self.prev_w) > 0.3 and abs(w) < JOY_NOISE_W and abs(v) > JOY_NOISE_V:
                self.get_logger().info('Boost nach Drehung')
                v_corrected += 0.07
            if self.v_actual < v - 0.05:
                v_corrected = max(v_corrected, v + 0.05)
            elif self.v_actual > v + 0.05:
                v_corrected = min(v_corrected, v - 0.05)
            if abs(v) > 0.01 and abs(v_corrected) < min_linear:
                v_corrected = min_linear * (1 if v > 0 else -1)
            v_corrected = max(min(v_corrected, MAX_LINEAR), -MAX_LINEAR)

        elif mode == 'TURN':
            # Nur Drehen (Vorwärtsfahrt bleibt 0)
            if abs(v) > JOY_NOISE_V:
                w_corrected = w * angular_scale * 0.5
            else:
                w_corrected = w * angular_scale

            # Mindestdrehung nur im sicheren TURN-Bereich
            if abs(w) >= TURN_ON and abs(w_corrected) < min_angular:
                w_corrected = min_angular * (1 if w > 0 else -1)
            elif abs(w) <= TURN_OFF:
                # Untere Schwelle: aktiv auf 0 bremsen
                w_corrected = 0.0

            w_corrected = max(min(w_corrected, MAX_ANGULAR), -MAX_ANGULAR)

        else:  # IDLE
            v_corrected = 0.0
            w_corrected = 0.0

        # Fallback: expliziter Null-Stopp bei ganz kleinen Werten
        if abs(v) < 0.01 and abs(w) < 0.01:
            v_corrected = 0.0
            w_corrected = 0.0
            self.get_logger().info('Stillstand erkannt → 0.0')

        new_cmd = Twist()
        new_cmd.linear.x = v_corrected
        new_cmd.angular.z = w_corrected
        self.cmd_vel_pub.publish(new_cmd)
        self.prev_w = w

    def _start_counter_pulse(self, sign: float):
        if self._pulse_timer is not None:
            self._pulse_timer.cancel()
            self._pulse_timer = None
            self._pulses_left = 0
        self._pulse_sign = sign
        self._pulses_left = max(1, int((COUNTER_MS / 1000.0) * COUNTER_HZ))
        period = 1.0 / COUNTER_HZ
        self._pulse_timer = self.create_timer(period, self._pulse_tick)
        self.get_logger().info('Gegendreh-Impuls gestartet')

    def _pulse_tick(self):
        if self._pulses_left <= 0:
            t = Twist(); t.linear.x = 0.0; t.angular.z = 0.0
            self.cmd_vel_pub.publish(t)
            if self._pulse_timer is not None:
                self._pulse_timer.cancel()
                self._pulse_timer = None
            # Sicherheit: noch einmal 0 senden und prev_w zurücksetzen
            z = Twist(); z.linear.x = 0.0; z.angular.z = 0.0
            self.cmd_vel_pub.publish(z)
            self.prev_w = 0.0
            self.get_logger().info('Gegendreh-Impuls fertig')
            return
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = self._pulse_sign * COUNTER_W
        self.cmd_vel_pub.publish(t)
        self._pulses_left -= 1


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelTransformer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

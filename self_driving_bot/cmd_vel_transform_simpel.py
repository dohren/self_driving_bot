#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# --- deine bisherigen Werte ---
TURN_THRESHOLD = 0.1
DRIVE_THRESHOLD = 0.05

DRIVE_SPEED = 0.45
TURN_SPEED  = 0.6

BOOST_AMOUNT = 0.2     # Zusatzgeschwindigkeit beim Start
BOOST_TIME   = 0.3     # Dauer in Sekunden

# --- neu: kurze Stopps für sauberes Nachjustieren ---
MODE_MAX_SEC_TURN  = 1.5   # max. ununterbrochene Drehdauer
MODE_MAX_SEC_DRIVE = 1.2   # max. ununterbrochene Fahrdauer
MODE_BREAK_SEC     = 0.4   # so lange 0/0 senden
SWITCH_BREAK_SEC   = 0.3   # kurzer Stopp bei TURN<->DRIVE Wechsel

class CmdVelMode(Node):
    def __init__(self):
        super().__init__('cmd_vel_mode')
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_transform', 10)

        self.last_mode = "IDLE"
        self.boost_active_until = 0.0

        # neu
        self._mode_started_at = None
        self._block_until = 0.0

        self.get_logger().info("CmdVelMode aktiv (Boost + kurze Stopps für Planner)")

    def compute_target(self, v_in: float, w_in: float):
        if abs(w_in) > TURN_THRESHOLD and abs(w_in) >= abs(v_in):
            return 0.0, TURN_SPEED * (1 if w_in > 0 else -1), "TURN"
        elif abs(v_in) > DRIVE_THRESHOLD:
            return DRIVE_SPEED * (1 if v_in > 0 else -1), 0.0, "DRIVE"
        else:
            return 0.0, 0.0, "IDLE"

    def apply_boost(self, v: float, w: float, mode: str):
        now = time.time()
        if self.last_mode == "IDLE" and mode != "IDLE":
            self.boost_active_until = now + BOOST_TIME
            self.get_logger().info("Boost gestartet")
        if now < self.boost_active_until:
            if mode == "DRIVE":
                v += BOOST_AMOUNT * (1 if v > 0 else -1)
            elif mode == "TURN":
                w += BOOST_AMOUNT * (1 if w > 0 else -1)
        return v, w

    def _publish_zero(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.pub.publish(t)

    def cmd_vel_callback(self, msg: Twist):
        now = time.time()

        # Blockphase: strikt 0/0
        if now < self._block_until:
            self._publish_zero()
            return

        v_in = msg.linear.x
        w_in = msg.angular.z

        v_out, w_out, mode = self.compute_target(v_in, w_in)

        # --- kurze Stopps: Laufzeit-Begrenzung je Modus ---
        if mode != "IDLE":
            # Startzeit setzen bei neuem Modus
            if mode != self.last_mode:
                # Mode-Wechsel TURN <-> DRIVE? → kurzer Stopp
                if (self.last_mode in ("TURN", "DRIVE")) and (mode in ("TURN", "DRIVE")):
                    self._block_until = now + SWITCH_BREAK_SEC
                    self._publish_zero()
                    self.last_mode = "IDLE"   # nach Break als Stillstand werten
                    return
                self._mode_started_at = now
            else:
                # gleiche Mode weiter: Dauer prüfen
                max_sec = MODE_MAX_SEC_TURN if mode == "TURN" else MODE_MAX_SEC_DRIVE
                if self._mode_started_at and (now - self._mode_started_at) > max_sec:
                    self._block_until = now + MODE_BREAK_SEC
                    self._publish_zero()
                    self.last_mode = "IDLE"
                    return
        else:
            # im Idle zählt die Zeit nicht
            self._mode_started_at = None

        # Boost anwenden (deine Logik)
        v_out, w_out = self.apply_boost(v_out, w_out, mode)

        # Log
        self.get_logger().info(
            f"IN  v={v_in:.3f}, w={w_in:.3f}  →  OUT v={v_out:.3f}, w={w_out:.3f}"
        )

        # Publish
        out = Twist()
        out.linear.x = v_out
        out.angular.z = w_out
        self.pub.publish(out)

        self.last_mode = mode


def main():
    rclpy.init()
    node = CmdVelMode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

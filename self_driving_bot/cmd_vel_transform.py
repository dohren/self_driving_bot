import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class CmdVelTransformer(Node):
    def __init__(self):
        super().__init__('cmd_vel_transformer')

        # Parameter: Encoder-Konfiguration
        self.declare_parameter('ticks_meter', 3400.0)
        self.ticks_meter = self.get_parameter('ticks_meter').value

        # Aktuelle Encoder-Werte
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = None
        self.prev_right_ticks = None

        # Geschwindigkeit (berechnet)
        self.v_actual = 0.0

        # Zeit-Tracking
        self.prev_time = self.get_clock().now()

        # Subscriber
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int32, '/left_ticks', self.left_callback, 10)
        self.create_subscription(Int32, '/right_ticks', self.right_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_transform', 10)

    def left_callback(self, msg):
        self.left_ticks = msg.data
        self.update_velocity()

    def right_callback(self, msg):
        self.right_ticks = msg.data
        self.update_velocity()

    def update_velocity(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9  # Sekunden

        if dt == 0:
            return  # Schutz vor Division durch 0

        if self.prev_left_ticks is not None and self.prev_right_ticks is not None:
            delta_left = self.left_ticks - self.prev_left_ticks
            delta_right = self.right_ticks - self.prev_right_ticks

            # Mittlere Strecke beider Räder
            delta_distance = (delta_left + delta_right) / 2.0
            self.v_actual = (delta_distance / self.ticks_meter) / dt

        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks
        self.prev_time = now

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # Berechne Korrektur (rudimentär)
        v_corrected = v
        w_corrected = w

        if abs(v) < 0.01 and abs(w) < 0.01:
            # Expliziter Null-Stopp
            v_corrected = 0.0
            w_corrected = 0.0
            self.get_logger().info("Stillstand erkannt → sende explizit 0.0")
        else:
            # Dynamische Anpassung je nach tatsächlicher Bewegung
            if self.v_actual < v - 0.05:
                v_corrected = v + 0.05
            elif self.v_actual > v + 0.05:
                v_corrected = v - 0.05

        # Konfigurierbare Parameter für Dämpfung
        linear_scale = 0.5   # z. B. 50 % der linearen Geschwindigkeit
        angular_scale = 0.3  # z. B. 30 % der Drehgeschwindigkeit

        min_linear = 0.55    # minimale Geschwindigkeit, um nicht zu stehen
        min_angular = 0.7

        MAX_LINEAR = 0.2     # statt 0.55 – sonst rast er
        MAX_ANGULAR = 0.4  

        v_corrected = v * linear_scale

        # Boost bei Wechsel von Drehen zu Geradeausfahrt
        if hasattr(self, 'prev_w'):
            if abs(self.prev_w) > 0.3 and abs(w) < 0.05 and abs(v) > 0.05:
                self.get_logger().info("Wechsel von Drehung zu Geradeausfahrt – Boost aktiv")
                v_corrected += 0.07 

        # Nur minimale lineare Geschwindigkeit erzwingen, wenn Bewegung geplant
        if abs(v) > 0.01 and abs(v_corrected) < min_linear:
            v_corrected = min_linear * (1 if v > 0 else -1)

        # Begrenzung
        v_corrected = max(min(v_corrected, MAX_LINEAR), -MAX_LINEAR)

        # Skaliere angular.z
        # Drehen nur reduzieren, wenn wir uns auch vorwärts bewegen
        if abs(v) > 0.05:
            w_corrected = w * angular_scale * 0.5  # noch weiter reduzieren beim Fahren
        else:
            w_corrected = w * angular_scale

        # Mindestdrehgeschwindigkeit nur, wenn keine Vorwärtsfahrt
        if abs(v) < 0.05 and abs(w) > 0.01 and abs(w_corrected) < min_angular:
            w_corrected = min_angular * (1 if w > 0 else -1)

        # Begrenzung
        w_corrected = max(min(w_corrected, MAX_ANGULAR), -MAX_ANGULAR)

        # Begrenzung
        w_corrected = max(min(w_corrected, MAX_ANGULAR), -MAX_ANGULAR)

        # Neue Nachricht veröffentlichen
        new_cmd = Twist()
        new_cmd.linear.x = v_corrected
        new_cmd.angular.z = w_corrected
        self.cmd_vel_pub.publish(new_cmd)

        self.prev_w = w


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

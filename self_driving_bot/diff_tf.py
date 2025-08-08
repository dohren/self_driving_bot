
import rclpy
from rclpy.node import Node
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int32

NS_TO_SEC = 1000000000

class DiffTf(Node):
    """
       diff_tf.py - follows the output of a wheel encoder and
       creates tf and odometry messages.
    """

    def __init__(self):
        super().__init__('diff_tf2')
        self.get_logger().info(f"-I- diff_tf started")

        # Parameters
        self.rate_hz = self.declare_parameter("rate_hz", 10.0).value
        self.ticks_meter = float(self.declare_parameter('ticks_meter', 3400).value)
        self.base_width = float(self.declare_parameter('base_width', 0.6).value)

        self.base_frame_id = self.declare_parameter('base_frame_id', 'base_footprint').value
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value

        # Timer
        self.create_timer(1.0 / self.rate_hz, self.update)

        # Internal data
        self.enc_left = None
        self.enc_right = None
        self.left = 0.0
        self.right = 0.0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0
        self.dr = 0.0
        self.then = self.get_clock().now()

        # Subscriptions and Publishers
        self.create_subscription(Int32, "left_ticks", self.lwheel_callback, 10)
        self.create_subscription(Int32, "right_ticks", self.rwheel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.odom_broadcaster = TransformBroadcaster(self)

    def update(self):
        now = self.get_clock().now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.nanoseconds / NS_TO_SEC

        # Odometry calculation
        d_left = (self.left - self.enc_left) / self.ticks_meter if self.enc_left is not None else 0
        d_right = (self.right - self.enc_right) / self.ticks_meter if self.enc_right is not None else 0

        self.enc_left, self.enc_right = self.left, self.right

        d = (d_left + d_right) / 2
        th = (d_right - d_left) / self.base_width

        self.dx = d / elapsed
        self.dr = th / elapsed

        if d != 0:
            self.x += cos(self.th) * d
            self.y += sin(self.th) * d
        if th != 0:
            self.th += th

        # Publish odom and tf information
        quaternion = Quaternion()
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = now.to_msg()
        transform_stamped_msg.header.frame_id = self.odom_frame_id
        transform_stamped_msg.child_frame_id = self.base_frame_id
        transform_stamped_msg.transform.translation.x = self.x
        transform_stamped_msg.transform.translation.y = self.y
        transform_stamped_msg.transform.rotation = quaternion

        self.odom_broadcaster.sendTransform(transform_stamped_msg)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.angular.z = self.dr

        self.odom_pub.publish(odom)

    def lwheel_callback(self, msg):
        self.left = float(msg.data)

    def rwheel_callback(self, msg):
        self.right = float(msg.data)

def main(args=None):
    rclpy.init(args=args)
    try:
        diff_tf = DiffTf()  
        rclpy.spin(diff_tf)
    except rclpy.exceptions.ROSInterruptException:
        pass

    diff_tf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
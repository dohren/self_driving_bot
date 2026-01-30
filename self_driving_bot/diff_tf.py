#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

NS_TO_SEC = 1e9


class OdomFromTicksTF(Node):
    def __init__(self):
        super().__init__("odom_from_ticks")

        # ===== Parameter (später per YAML) =====
        self.WHEEL_BASE = 0.60           # Meter
        self.MPT_L = 0.000319            # meter pro tick (links)
        self.MPT_R = 0.000294            # meter pro tick (rechts)

        self.odom_frame = "odom"
        self.base_frame = "base_footprint"

        # ===== ROS I/O =====
        self.sub_l = self.create_subscription(
            Int32, "/left_ticks", self.left_cb, 10
        )
        self.sub_r = self.create_subscription(
            Int32, "/right_ticks", self.right_cb, 10
        )

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ===== Encoder State =====
        self.l = None
        self.r = None
        self.prev_l = None
        self.prev_r = None
        self.new_l = False
        self.new_r = False

        # ===== Pose =====
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_time = self.get_clock().now()

        self.get_logger().info("OdomFromTicksTF started (ROS TF compatible)")

    # =============================
    # Encoder Callbacks
    # =============================
    def left_cb(self, msg: Int32):
        self.l = msg.data
        self.new_l = True
        self.update()

    def right_cb(self, msg: Int32):
        self.r = msg.data
        self.new_r = True
        self.update()

    # =============================
    # Odometrie Update
    # =============================
    def update(self):
        if self.l is None or self.r is None:
            return

        if self.prev_l is None:
            self.prev_l = self.l
            self.prev_r = self.r
            self.last_time = self.get_clock().now()
            self.new_l = False
            self.new_r = False
            return

        # nur integrieren, wenn beide neu
        if not (self.new_l and self.new_r):
            return

        dl_ticks = self.l - self.prev_l
        dr_ticks = self.r - self.prev_r

        self.prev_l = self.l
        self.prev_r = self.r
        self.new_l = False
        self.new_r = False

        dl = dl_ticks * self.MPT_L
        dr = dr_ticks * self.MPT_R

        ds = 0.5 * (dl + dr)
        dth = (dr - dl) / self.WHEEL_BASE

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / NS_TO_SEC
        self.last_time = now

        # ===== Integration (korrekt) =====
        self.x += ds * math.cos(self.th + 0.5 * dth)
        self.y += ds * math.sin(self.th + 0.5 * dth)
        self.th += dth

        # ===== Quaternion =====
        q = quaternion_from_euler(0.0, 0.0, self.th)
        quat = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # ===== TF: odom → base_footprint =====
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.rotation = quat
        self.tf_broadcaster.sendTransform(tf_msg)

        # ===== Odometry =====
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quat

        if dt > 0.0:
            odom.twist.twist.linear.x = ds / dt
            odom.twist.twist.angular.z = dth / dt

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = OdomFromTicksTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

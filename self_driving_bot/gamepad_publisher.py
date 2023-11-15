#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from time import sleep
from evdev import InputDevice, categorize, ecodes, list_devices
from threading import Thread
import sys

from geometry_msgs.msg import Twist


class GamepadNode(Node):

    maxThrottle = 0.4
    status_steering = 0.0
    status_throttle = 0.0
    joyVal = { 'steering': 0., 'throttle': 0., 'X': 0, 'A': 0, 'B': 0, 'Y': 0, 'select': 0, 'start': 0}
    mapping = {589825: 'X', 589826: 'A', 589827: 'B', 589828: 'Y', 589833: 'select', 589834: 'start'}

    def __init__(self):
        super().__init__("gamepad")

        self.setGamepadDevice()
        input_thread = Thread(target=self.check_for_input, daemon=True)
        input_thread.start() 
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer_ = self.create_timer(0.02, self.publish_speed)
        self.get_logger().info("Gempad Publisher has been started")


    def setGamepadDevice(self):
        self.gamepad = None
        devices = [InputDevice(path) for path in list_devices()]
        for device in devices:
            print(device)
            if "Controller" in device.name:
                self.gamepad = device
                print("test")
        if self.gamepad == None:
           sys.exit("Controller not found!")

    def transform(self, value):
        if value == 0: return -1.
        if value == 128: return 0.
        if value == 255: return 1.

    def check_for_input(self):
        for event in self.gamepad.async_read_loop():
            if event.type == 3: # arrows
                if event.code == 0:
                        self.joyVal['steering'] = self.transform(event.value)
                elif event.code == 1:
                        self.joyVal['throttle'] = self.transform(event.value)
            if event.type == 4: # buttons
                button = self.mapping[event.value]
                self.joyVal[button] = 0 if self.joyVal[button] == 1 else 1
                print(self.joyVal[button])  


    def publish_speed(self):

        steering = self.joyVal['steering']
        throttle = self.joyVal['throttle'] * self.maxThrottle

        if self.status_throttle != throttle or self.status_steering != steering:
            self.status_throttle = throttle
            self.status_steering = steering
            msg = Twist()
            msg.linear.x = -throttle;
            msg.angular.z = -steering;

            if steering != 0:
                msg.linear.x = -throttle;

            #print(str(throttle) + " " + str(steering))
            self.publisher_.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = GamepadNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

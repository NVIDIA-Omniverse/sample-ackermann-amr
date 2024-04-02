#!/usr/bin/env python3

# Copyright (c) 2020-2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import rclpy
from rclpy.node import Node
from pynput import keyboard
from ackermann_msgs.msg import AckermannDriveStamped

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        # Create the publisher and message
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        self.drive_msg = AckermannDriveStamped()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_cmd)
	
    def on_press(self, key):
        try:
            # 1.5.3 Create ROS Messages based on keys pressed
            pass
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            # 3. If no keys are pressed, stop the car
            if key.char in ['w', 's']:
                self.drive_msg.drive.speed = 0.0
            elif key.char in ['a', 'd']:
                self.drive_msg.drive.steering_angle = 0.0
        except AttributeError:
            pass

    def publish_cmd(self):
        # Publish the Message
        self.drive_msg.header.frame_id = "f1_tenth"
        self.drive_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.drive_msg)

def main(args=None):
    rclpy.init(args=args)
    keyboard_teleop_node = KeyboardTeleopNode()
    keyboard_teleop_node.get_logger().info('Listening to keyboard. Use WASD to drive!')
    try:
        rclpy.spin(keyboard_teleop_node)
    except KeyboardInterrupt:
        keyboard_teleop_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

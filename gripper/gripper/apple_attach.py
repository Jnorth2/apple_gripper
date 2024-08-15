#!/usr/bin/env python3

import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool


class AppleAttach(Node):

    def __init__(self):
        super().__init__('apple_attach')
        self.subscription = self.create_subscription(String, 'gripper/motor/position', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.previous_position = 0

        self.publisher = self.create_publisher(Bool, '/gripper/apple_grasp', 10)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)

        data = re.search("(?:-*\d+)(\.*)(\d*)(?!.*\d)", msg.data)
        data = float(data.group(0))
        if data<-1500 and data == self.previous_position:
            msg = Bool()
            msg.data = True
            self.publisher.publish(msg)
        self.previous_position = data

        



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = AppleAttach()

    rclpy.spin(minimal_subscriber)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
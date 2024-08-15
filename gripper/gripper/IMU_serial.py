#!/usr/bin/env python3

# This node communicates with the arduino via Pyserial to recieve and publish IMU data

# ROS imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool

# standard imports
import numpy as np
import serial
from serial import SerialException
import re

import numpy as np # Scientific computing library for Python


def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return w, x, y, z

class LFD_IMU(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("suction_gripper")

        # set up the publishers
        self.publisher = self.create_publisher(Quaternion, '/gripper/IMU', 1)
        # set up the publishers
        self.publisher2 = self.create_publisher(Bool, '/gripper/apple_grasp', 1)

        # set up Pyserial
        arduino_port = "/dev/ttyUSB0"
        baud = 9600
        self.my_serial = serial.Serial(arduino_port, baud)
        self.get_logger().info(f"Connected to serial port {arduino_port}")
        self.timer = self.create_timer(0.1, self.timer_callback) #to continuously read the sensors

        self.counter = 0
            
    

    def timer_callback(self):
        """read all of the messages in the serial"""

        while self.my_serial.in_waiting:
            line = self.my_serial.readline().decode('utf-8')[0:][:-2]
            data = re.findall("-*\d+\.\d*", line)
            msg = Quaternion()
            q = float(data[0]), float(data[1]), float(data[2]), float(data[3])

            q_x_90 = np.array([np.cos(np.pi * 3 / 4), np.sin(np.pi * 3 / 4), 0, 0])
            q = quaternion_multiply(q, q_x_90)


            msg.w, msg.x, msg.y, msg.z = q
            self.publisher.publish(msg)

            self.counter +=1
            if self.counter == 50:
                self.get_logger().info("bool published")
                msg = Bool()
                msg.data = True
                self.publisher2.publish(msg)




def main(args=None):
    # initialize
    rclpy.init(args=args)
    # instantiate the class
    my_imu = LFD_IMU()
    # hand control over to ROS2
    rclpy.spin(my_imu)
    # shutdown cleanly
    rclpy.shutdown()

if __name__ == "__main__":
    main()
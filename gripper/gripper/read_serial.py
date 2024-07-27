#!/usr/bin/env python3

# This node communicates with the arduino nano using pyserial to publish imu data

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # or a custom message type

# standard imports
import serial
from serial import SerialException
import re

class ReadSerial(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("suction_gripper")

        # set up the publishers
        self.publisher_imu = self.create_publisher(String, '/gripper/imu', 10)

        # set up Pyserial
        arduino_port = "/dev/ttyUSB0"
        baud = 9600

        try:
            self.my_serial = serial.Serial(arduino_port, baud)
            self.get_logger().info(f"Connected to serial port {arduino_port}")
            self.timer = self.create_timer(0.1, self.timer_callback) #to read the sensors
            self.get_logger().info("Publishing imu data")
        except SerialException as e:
            self.get_logger().info(e)
    

    def timer_callback(self):
        while self.my_serial.in_waiting:
            line = self.my_serial.readline().decode('utf-8')[0:][:-2]
            msg = String()
            msg.data = line
            self.publisher_imu.publish(msg)


def main(args=None):
    # initialize
    rclpy.init(args=args)
    # instantiate the class
    read_serial = ReadSerial()
    # hand control over to ROS2
    rclpy.spin(read_serial)
    # shutdown cleanly
    rclpy.shutdown()

if __name__ == "__main__":
    main()
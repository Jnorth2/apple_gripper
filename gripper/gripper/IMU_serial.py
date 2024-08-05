#!/usr/bin/env python3

# This node communicates with the arduino via Pyserial to recieve and publish IMU data

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# standard imports
import numpy as np
import serial
from serial import SerialException
import re

class LFD_IMU(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("suction_gripper")

        # set up the publishers
        self.publisher = self.create_publisher(String, '/gripper/IMU', 10)

        # set up Pyserial
        arduino_port = "/dev/ttyUSB1"
        baud = 9600
        try:
            self.my_serial = serial.Serial(arduino_port, baud)
            self.get_logger().info(f"Connected to serial port {arduino_port}")
            self.timer = self.create_timer(0.1, self.timer_callback) #to continuously read the sensors
        except SerialException as e:
            self.get_logger().info(f"Could not connect to serial port {arduino_port}.")
            print(e)
            # self.get_logger().info(e)
            
    

    def timer_callback(self):
        """read all of the messages in the serial"""
        # self.get_logger().info(self.my_serial.in_waiting)

        while self.my_serial.in_waiting:
            line = self.my_serial.readline().decode('utf-8')[0:][:-2]
            msg = String()
            msg.data = line
            self.publisher.publish(msg)




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
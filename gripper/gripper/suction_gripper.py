#!/usr/bin/env python3

# This node communicates with the openrb150 via Pyserial to control
    # the gripper - vacuum on/off, fingers open/close, mux on/off

# ROS imports
import rclpy
from rclpy.node import Node
from gripper_msgs.srv import GripperVacuum, GripperFingers, GripperMultiplexer
from std_msgs.msg import String  # or a custom message type

# standard imports
import numpy as np
import serial
from serial import SerialException
import re

class SuctionGripper(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("suction_gripper")

        # set up the publishers
        publisher_sc1 = self.create_publisher(String, '/gripper/pressure/sc1', 10)
        publisher_sc2 = self.create_publisher(String, '/gripper/pressure/sc2', 10)
        publisher_sc3 = self.create_publisher(String, '/gripper/pressure/sc3', 10)
        publisher_dist = self.create_publisher(String, '/gripper/distance', 10)
        publisher_current = self.create_publisher(String, '/gripper/motor/current', 10)
        publisher_pos = self.create_publisher(String, '/gripper/motor/position', 10)
        publisher_vel = self.create_publisher(String, '/gripper/motor/velocity', 10)
        self.publisher_list = [publisher_sc1, publisher_sc2, publisher_sc3, publisher_dist,
                               publisher_current, publisher_pos, publisher_vel]

        # set up the services
        self.vacuum_service = self.create_service(GripperVacuum, 'set_vacuum_status', self.vacuum_service_callback)
        self.fingers_service = self.create_service(GripperFingers, 'set_fingers_status', self.fingers_service_callback)
        self.multiplexer_service = self.create_service(GripperMultiplexer, 'set_multiplexer_status', self.multiplexer_service_callback)

        # set up Pyserial
        arduino_port = "/dev/ttyACM0"
        baud = 57600
        try:
            self.my_serial = serial.Serial(arduino_port, baud)
            self.get_logger().info(f"Connected to serial port {arduino_port}")
            self.real_arduino_flag = True
            self.timer = self.create_timer(0.1, self.timer_callback) #to read the sensors
        except SerialException as e:
            self.get_logger().info(f"Could not connect to serial port {arduino_port}. Using fake arduino hardware.")
            self.real_arduino_flag = False
            # self.get_logger().info(e)
            
        # class variables
        self.start_character = "<"
        self.end_character = ">"
        self.vacuum_on_command = "1"
        self.vacuum_off_command = "2"
        self.fingers_engaged_command = "3"
        self.fingers_disengaged_command = "4"
        self.multiplexer_on_command = "5"
        self.multiplexer_off_command = "6"
    

    def timer_callback(self):
        while self.my_serial.in_waiting:
            line = self.my_serial.readline().decode('utf-8')[0:][:-2]

            # if it has a tag to be published line ex. [Ch2]
            if re.search("\[Ch[0-9]*?\]", line):

                #extract tag and channel from message
                tag = re.search("\[Ch[0-9]*?\]", line)
                channel = int(tag.group(0)[3:-1])

                #publish to the correct topic
                publisher = self.publisher_list[channel]
                msg = String()
                msg.data = line
                publisher.publish(msg)

            # if it does not have a tag it is user feedback and printed
            else:
                self.get_logger().info(line)



    def vacuum_service_callback(self, request, response):
        """Callback function for the vacuum service. Uses the bool stored in set_vacuum
            to turn the vacuum on or off
        """
        # if the request is True, turn the vacuum on
        if request.set_vacuum:
            # if we're actually connected to an arduino, send the serial message. Otherwise just pretend
            self.get_logger().info("Sending request: gripper vacuum on")
            if self.real_arduino_flag:
                msg_str = self.start_character+self.vacuum_on_command+self.end_character
                self.my_serial.write(str(msg_str).encode())
                # self.read_serial()
        # if the request is False, turn the vacuum off
        elif not request.set_vacuum:
            self.get_logger().info("Sending request: gripper vacuum off")
            if self.real_arduino_flag:
                msg_str = self.start_character+self.vacuum_off_command+self.end_character
                self.my_serial.write(str(msg_str).encode())
                # self.read_serial()

        response.result = True
        return response
    
    def fingers_service_callback(self, request, response):
        """Callback function for the fingers service. Uses the bool stored in set_fingers
            to engage or disengage the grippers. This might be better as an action.
        """
        # if the request is True, engage the fingers
        if request.set_fingers:
            # like above, if we're actually connected to an arduino, send the serial message. Otherwise just pretend
            self.get_logger().info("Sending request: engage fingers")
            if self.real_arduino_flag:
                msg_str = self.start_character+self.fingers_engaged_command+self.end_character
                self.my_serial.write(str(msg_str).encode())
                # self.read_serial()
        # if the request is False, disengage the fingers
        elif not request.set_fingers:
            self.get_logger().info("Sending request: disengage fingers")
            if self.real_arduino_flag:
                msg_str = self.start_character+self.fingers_disengaged_command+self.end_character
                self.my_serial.write(str(msg_str).encode())
                # self.read_serial()

        response.result = True
        return response


    def multiplexer_service_callback(self, request, response):
        """Callback function for the multiplexer service. Uses the bool stored in set_multiplexer
            to engage or disengage the grippers. This might be better as an action.
        """
        # if the request is True, engage the multiplexer
        if request.set_multiplexer:
            # like above, if we're actually connected to an arduino, send the serial message. Otherwise just pretend
            self.get_logger().info("Sending request: begin multiplexing")
            if self.real_arduino_flag:
                msg_str = self.start_character+self.multiplexer_on_command+self.end_character
                self.my_serial.write(str(msg_str).encode())
                #read once as usual and once for each sensor
                # for i in range(5):
                    # self.read_serial()
        # if the request is False, disengage the multiplexer
        elif not request.set_multiplexer:
            self.get_logger().info("Sending request: end multiplexing")
            if self.real_arduino_flag:
                msg_str = self.start_character+self.multiplexer_off_command+self.end_character
                self.my_serial.write(str(msg_str).encode())
                # self.read_serial()

        response.result = True
        return response


def main(args=None):
    # initialize
    rclpy.init(args=args)
    # instantiate the class
    my_gripper = SuctionGripper()
    # hand control over to ROS2
    rclpy.spin(my_gripper)
    # shutdown cleanly
    rclpy.shutdown()

if __name__ == "__main__":
    main()
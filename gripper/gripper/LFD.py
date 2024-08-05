#!/usr/bin/env python3

# This node interacts with the user, starts the pic sequence,
    # and saves the appropirate metadata as well as ros bags.
    # This is the main script for LFD data collection.

# ROS imports
import rclpy
from rclpy.node import Node
from gripper_msgs.srv import GripperFingers, GripperVacuum, GripperMultiplexer

# standard imports
import os
import subprocess
import time
import signal
import datetime


class LFDUser(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("user")
        self.get_logger().info("Started user node")

        # set up service clients
        self.vacuum_service_client = self.create_client(GripperVacuum, 'set_vacuum_status')
        self.get_logger().info("Waiting for gripper vaccuum server")
        self.vacuum_service_client.wait_for_service()

        self.fingers_service_client = self.create_client(GripperFingers, 'set_fingers_status')
        self.get_logger().info("Waiting for gripper finger server")
        self.fingers_service_client.wait_for_service()

        self.multiplexer_service_client = self.create_client(GripperMultiplexer, 'set_multiplexer_status')
        self.get_logger().info("Waiting for multiplexer server")
        self.multiplexer_service_client.wait_for_service()
    
    ##  ------------- SERVICE CLIENT CALLS ------------- ##
    def send_vacuum_request(self, vacuum_status):
        """Function to call gripper vacuum service.
            Inputs - vacuum_status (bool): True turns the vacuum on, False turns it off
        """
        # build the request
        request = GripperVacuum.Request()
        request.set_vacuum = vacuum_status
        # make the service call (asynchronously)
        self.vacuum_response = self.vacuum_service_client.call_async(request)

    def send_fingers_request(self, fingers_status):
        """Function to call gripper fingers service.
            Inputs - fingers_status (bool): True engages the fingers, False disengages
        """
        # build the request
        request = GripperFingers.Request()
        request.set_fingers = fingers_status
        # make the service call (asynchronously)
        self.fingers_response = self.fingers_service_client.call_async(request)

    def send_multiplexer_request(self, multiplexer_status):
        """Function to call gripper fingers service.
            Inputs - fingers_status (bool): True publishes the data, false does not
        """
        # build the request
        request = GripperMultiplexer.Request()
        request.set_multiplexer = multiplexer_status
        # make the service call (asynchronously)
        self.multiplexer_response = self.multiplexer_service_client.call_async(request)

    def datetime_simplified(self):
        """Convenient method to adapt datetime for file naming convention"""
        year = datetime.datetime.now().year
        month = datetime.datetime.now().month
        day = datetime.datetime.now().day
        hour = datetime.datetime.now().hour
        minute = datetime.datetime.now().minute

        day = str(year) + str(month//10) + str(month%10) + str(day)
        time = str(hour//10) + str(hour%10) + str(minute//10) + str(minute%10)

        return(day)
    
    def proxy_pick_sequence(self):
        """Function that collects data for a pick"""
        # start publishing multiplexer topics (suction cups, distance, motor feedback)
        self.send_multiplexer_request(multiplexer_status=True)
        self.get_logger().info("Multiplexing...")

        # start rosbag recording (edit topics array to enable/disable camera recording)
        rosbag_number = 1
        while os.path.exists('/apple_gripper/gripper/data/bags/LFD' + self.datetime_simplified() + "_" + str(rosbag_number)):
            rosbag_number += 1
        self.get_logger().info("Starting rosbag")
        file_name = '/apple_gripper/gripper/data/bags/' + self.datetime_simplified() + "_" + str(rosbag_number)
        topics = ['/gripper/distance', '/gripper/pressure/sc1', '/gripper/pressure/sc2', '/gripper/pressure/sc3', '/gripper/motor/current', '/gripper/motor/position', '/gripper/motor/velocity', '/gripper/camera', '/gripper/imu', '/gripper/force_torque']
        cmd = 'ros2 bag record -o ' + file_name + ' ' + ' '.join(topics)
        pro = subprocess.Popen(cmd, stdout=subprocess.PIPE, stdin=subprocess.PIPE,
                        shell=True, preexec_fn=os.setsid) 
        
        input("Press enter to turn on vacuum: ")

        # turn on vacuum - TODO actuate this with sensor feedback
        self.send_vacuum_request(vacuum_status=True)
        self.get_logger().info("Gripper vacuum on")

        input("Press enter to close fingers: ")

        # engage fingers - TODO actuate this with sensor feedback
        self.send_fingers_request(fingers_status=True)
        self.get_logger().info("Fingers engaged")

        input("Press enter to open fingers: ")

        # disengage fingers - GRIPPER
        self.send_fingers_request(fingers_status=False)
        self.get_logger().info("Fingers disengaged")

        input("Press enter to turn off vaccuum: ")

        # turn off vacuum - GRIPPER
        self.send_vacuum_request(vacuum_status=False)
        self.get_logger().info("Gripper vacuum off")


        # stop rosbag recording
        os.killpg(os.getpgid(pro.pid), signal.SIGTERM)
        time.sleep(2)
        self.get_logger().info("Rosbag stopped")

        # stop multiplexing
        self.send_multiplexer_request(multiplexer_status=False)
        self.get_logger().info("Not multiplexing...")

        return


def main(args=None):
    # initialize rclpy
    rclpy.init(args=args)
    # instantiate the class
    my_user = LFDUser()
    # run the proxy pick sequence
    input("Hit enter to start proxy test: ")
    my_user.proxy_pick_sequence()

if __name__ == "__main__":
    main()
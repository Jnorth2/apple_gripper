#!/usr/bin/env python3

import numpy as np
#import pandas as pd
import scipy
from scipy.ndimage import median_filter
import os
import time
import json

# ROS2 imports
import rclpy
import rclpy.duration
import rclpy.logging
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from ament_index_python.packages import get_package_share_directory

# ROS2 transforms
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

# Interfaces
from std_srvs.srv import Trigger, Empty
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from gripper_msgs.srv import GripperVacuum, GripperFingers, GripperMultiplexer, SetArmGoal, GetArmPosition

class GraspController(Node):
    def __init__(self):
        super().__init__('grasp_controller')

        # Services
        # self.text_in_rviz_srv = self.create_service(TextInRviz, 'place_text_in_rviz', self.toy_problem_callback)
        self.graps_apple_srv = self.create_service(Empty, 'grasp_apple', self.grasp_apple_callback)

        # ----- SERVICE CLIENTS TO OPERATE GRIPPER -----
        # These services are SERVED by the node "suction_gripper.py"
        self.vacuum_service_client = self.create_client(GripperVacuum, 'set_vacuum_status')
        while not self.vacuum_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for gripper vaccuum server")

        self.fingers_service_client = self.create_client(GripperFingers, 'set_fingers_status')
        while not self.fingers_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for gripper finger server")
        
        # ------ SERVICE CLIENTS TO OPERATE ARM -----        
        # These services are SERVED by the node "arm_control.py"
        self.get_pos_service_client = self.create_client(GetArmPosition, 'get_arm_position')
        while not self.get_pos_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for position server")
        
        self.arm_service_client = self.create_client(SetArmGoal, 'set_arm_goal')
        while not self.arm_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arm service")
    

        # Create publisher
        self.marker_text_publisher = self.create_publisher(Marker, 'caption', 10)

    ## ---------------  SERVICE CALLBACKS (SERVER)-------------- ##
    def toy_problem_callback(self, request, response):
        """
        """
        
        # Toy problem to display a text in rviz
        caption = Marker()
        caption.header.frame_id = 'base_link'
        caption.header.stamp = self.get_clock().now().to_msg()
        caption.id = 1        
        caption.type = caption.TEXT_VIEW_FACING       
        #caption.action = Marker.ADD 

        # Set pose
        caption.pose.position.x = 0.0
        caption.pose.position.y = 0.0
        caption.pose.position.z = 1.25

        # Set color
        caption.color.r = 1.0
        caption.color.g = 1.0
        caption.color.b = 1.0
        caption.color.a = 1.0   # Fully opaque

        # Set scale
        caption.scale.z = 0.10   # Text height

        caption.text = request.message                

        self.marker_text_publisher.publish(caption)

        response.success = True

        return response


    def grasp_apple_callback(self, request, response):

        # Linear approach to the fruit
        print('before')
        self.send_position_request()
        print('after')


        # Swith vacuum on
        self.send_vacuum_request(True)
        time.sleep(0.5)

        # Air pressure servoing


        # Close fingers
        self.send_fingers_request(True)
        time.sleep(3)

        # TODO: These should be commented out and replaced with Miranda's service calls
        self.send_fingers_request(False)
        time.sleep(0.5)
        self.send_vacuum_request(False)

        return response


    ## --------------- SERVICE CLIENT CALLS (REQUESTS) -------------- ##
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

    def send_position_request(self, frame="world"):
        """Method to get the end effector position
        Inputs: frame (string): Desired frame
        """

        # build request
        request = GetArmPosition.Request()
        request.frame = frame

        # make the service call
        self.current_arm_position = self.get_pos_service_client.call_async(request).result()
        self.get_logger().info(self.current_arm_position)


    # def grasp_apple_callback_pending(self, request, response):
        # # Air-Pressure servoing Parameters
        # MAX_ATTEMPTS = 5
        # PRESSURE_READINGS = 10
        # ENGAGEMENT_THRESHOLD = 600    # Units hPa
        #
        # # Approach fruit in straight line
        # # TODO: linear approach and turn vacuum on
        # # TODO: Subscribe to the toF distance
        # self.get_logger().info(f'Initial linear approach to apple')
        #
        # # Control loop
        # cnt = 0
        # while cnt < MAX_ATTEMPTS:
        #
        #     # Air Pressure Readings
        #     ps1_list = []
        #     ps2_list = []
        #     ps3_list = []
        #     for i in range(PRESSURE_READINGS):
        #         # TODO: Have the gripper publishing the air-pressure readings
        #         # TODO: Have this node subscribed to the air-pressure readings
        #         ps1_list.append(TODO)
        #         ps2_list.append(TODO)
        #         ps3_list.append(TODO)
        #     ps1_mean = np.mean(ps1_list)
        #     ps2_mean = np.mean(ps2_list)
        #     ps3_mean = np.mean(ps3_list)
        #     self.get_logger().info(f'Mean Air Pressure Readings: {int(ps1_mean)}, {int(ps2_mean)}, {int(ps3_mean)}')
        #
        #     if ps1_mean < ENGAGEMENT_THRESHOLD and ps2_mean < ENGAGEMENT_THRESHOLD and ps3_mean < ENGAGEMENT_THRESHOLD:
        #         self.get_logger().info(f'All suction cups are engaged')
        #         break
        #
        #     # Adjust Pose
        #     self.get_logger().info(f'Adjusting angular pose of gripper')
        #     theta, omega = self.rotation_axis_angle(ps1_mean, ps2_mean, ps3_mean)
        #     cr_x, cr_y =

    def rotation_axis_angle(gp1, gp2, gp3):
        """

        """
        
        def vector_math(pressure_array):
            return v_sum
        
        def axis_of_rotation(v_sum):
            return axis

        def angle_math(v_sum):
            theta = np.arctan2([v_sum[1]], [v_sum[0]])
            return theta[0]

        def speed_calc(v_sum):
            return speed

        pressure = [gp1, gp2, gp3]

        neg_error1 = pressure[1] - 223 #(0.50 * pressure[1])
        neg_error2 = pressure[2] - 223 #(0.50 * pressure[2])
        pos_error1 = pressure[1] + 223 #(0.50 * pressure[1])
        pos_error2 = pressure[2] + 223 #(0.50 * pressure[2])

        if pressure[0] > 400 or pressure[1] > 400 or pressure[2] > 400:
            v_sum = vector_math(pressure)
            axis = axis_of_rotation(v_sum)
            omega = angle_math(v_sum)
            theta = np.linalg.norm(v_sum)
            speed = speed_calc(v_sum)


        elif pressure[0] <= 400 and pressure[1] <= 400 and pressure[2] <= 400:
            if neg_error1 <= pressure[0] <= pos_error1:
                if neg_error2 <= pressure[0] <= pos_error2:
                    if neg_error1 <= pressure[2] <= pos_error1:
                        print(f'All 3 suction cups engaged!')
                        omega = 0
                        theta = 0
                            
                    else:
                        v_sum = vector_math(pressure)
                        axis = axis_of_rotation(v_sum)
                        omega = angle_math(v_sum)
                        theta = np.linalg.norm(v_sum)
                        speed = speed_calc(v_sum)
                        

                else: 
                    v_sum = vector_math(pressure)
                    axis = axis_of_rotation(v_sum)
                    omega = angle_math(v_sum)
                    theta = np.linalg.norm(v_sum)
                    speed = speed_calc(v_sum)

            else:
                v_sum = vector_math(pressure)
                axis = axis_of_rotation(v_sum)
                omega = angle_math(v_sum)
                theta = np.linalg.norm(v_sum)
                speed = speed_calc(v_sum)




        return theta, omega





def main():
    rclpy.init()

    grasp_controller = GraspController()

    # Use a SingleThreadedExecutor to handle the callbacks
    executor = SingleThreadedExecutor()
    executor.add_node(grasp_controller)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        grasp_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
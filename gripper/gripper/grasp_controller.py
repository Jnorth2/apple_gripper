#!/usr/bin/env python3

import numpy as np
#import pandas as pd
import scipy
from scipy.ndimage import median_filter
import os
import time
import json
from collections import deque

# ROS2 imports
import rclpy
import rclpy.duration
import rclpy.logging
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory

# ROS2 transforms
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

# Interfaces
from std_srvs.srv import Trigger, Empty
from geometry_msgs.msg import Vector3, WrenchStamped, Twist, TwistStamped, TransformStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool, String, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from gripper_msgs.srv import GripperVacuum, GripperFingers, GripperMultiplexer, SetArmGoal, GetArmPosition

# Self developed 
from air_functions import * 

class GraspController(Node):
    def __init__(self):
        super().__init__('grasp_controller')

        # Specify reentrant callback group 
        r_callback_group = ReentrantCallbackGroup()
        
        # Topic publishers
        self.grasp_servo_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)        
        self.marker_text_publisher = self.create_publisher(Marker, 'caption', 10)

        # Topic subscriber
        self.tof_subscriber = self.create_subscription(String, '/gripper/distance', self.tof_process, 10)     
        self.air_press_subscriber = self.create_subscription(Float32MultiArray, '/gripper/pressure', self.air_press_process, 10)
        
        # Services
        # self.text_in_rviz_srv = self.create_service(TextInRviz, 'place_text_in_rviz', self.toy_problem_callback)
        self.graps_apple_srv = self.create_service(Trigger, 'grasp_apple', self.grasp_apple_callback, callback_group=r_callback_group)
                                  
        # Service clients
        # Theses services are providede by the node 'suction_gripper.py'
        self.vacuum_service_client = self.create_client(GripperVacuum, 'set_vacuum_status')
        while not self.vacuum_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for gripper vaccuum server")
        

        self.fingers_service_client = self.create_client(GripperFingers, 'set_fingers_status')
        while not self.fingers_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for gripper finger server")
        

        self.get_logger().info("All services Available!")
        
        # Recurring method
        self.timer = self.create_timer(0.01, self.timer_callback)
        

        # Variables        
        self.running = False
        self.tof_distance = 200        
        self.move_flag = False
        self.rate = self.create_rate(1)
        self.fingers_previous_action = "close"
        # Instantiate classes
        self.air_moving_avg = MovingAverage(10)     
        self.tof_moving_avg = MovingAverage(10)

        self.air_averages = [1000, 1000, 1000]

        # Parameters    
        self.VACUUM_TRIGGER_DISTANCE = 100   
        self.STOP_DISTANCE = 40   
        self.ENGAGEMENT_THRESHOLD = 600
        

     ## ---  SERVICE CALLBACKS (SERVER)
    def grasp_apple_callback(self, request, response):

        self.get_logger().info("Activating grasping node")
        self.move_flag = True
        self.running = True
        self.get_logger().info("Making sure fingers are open")
        if self.fingers_previous_action == "close":
            self.send_fingers_request(False)
        self.fingers_previous_action = "open"

        try:
            while rclpy.ok() and self.move_flag:
                self.get_logger().info("Servoing arm")   
                self.rate.sleep()
                
                # --- Step 1: Sense
                # Sense tof distance
                if self.tof_distance < self.VACUUM_TRIGGER_DISTANCE:
                    self.send_vacuum_request(True)
                else:
                    # self.send_vacuum_request(False)
                    pass
                # Sense air pressures
                thr = self.ENGAGEMENT_THRESHOLD
                scA = self.air_averages[0]
                scB = self.air_averages[1]
                scC = self.air_averages[2]
                if scA < thr and scB < thr and scC < thr:
                    self.get_logger().info("All suction cups engaged")
                    pass

                # --- Step 2: Adjust pose
                # Axis and angle
                axis, mag = axis_angle_rotation(self.air_averages)
                # Center of rotation
                x,y = center_of_rotation(self.air_averages)           
                self.get_logger().info(f"Center of Rotation {x}, {y}")    
                

              
                
        except Exception as e:
            self.get_logger().error(f"Error during grasping: {str(e)}")
        finally:
            self.move_flag = False
            self.running = False
            self.get_logger().info("Deploying fingers")            
            if self.fingers_previous_action == "open":
                self.send_fingers_request(True)
            self.fingers_previous_action = "close"
            
        
        return response
    
    
    ## --- SERVICE CLIENT CALLS (REQUESTS FROM THIS NODE)
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
    
    
    ## --- RUNNING FUNCTION TO CONTROL THE FLOW OF THE SERVICE 
    def timer_callback(self):
        
        # Create a Twist message for moving the arm
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tool0"

        if self.running:

            # ToF joystick
            # if self.tof_distance > self.STOP_DISTANCE:

            #     error = 120 - self.tof_distance
            #     vel = error/1000                
            #     msg.twist.linear.z = vel

            thr = self.ENGAGEMENT_THRESHOLD
            scA = self.air_averages[0]
            scB = self.air_averages[1]
            scC = self.air_averages[2]

            # if self.tof_distance > self.STOP_DISTANCE:                  
                # msg.twist.linear.z = 0.2

            if scA > thr and scB > thr and scC > thr:
                msg.twist.linear.z = 0.1
            
            elif scA < thr or scB < thr or scC < thr:
                msg.twist.linear.z = 0.0
                self.move_flag = False

            # else:
            #     msg.twist.linear.z = 0.0
            #     self.move_flag = False
        
        else:
                msg.twist.linear.z = 0.0
                self.move_flag = False


        # Publish the Twist message
        self.grasp_servo_publisher.publish(msg)


    ## --- FUNCTIONS TRIGGERED BY THE TOPIC SUBSCRIBERS
    def tof_process(self, msg):
        """ This is a SUBSCRIBER to a topic
            The Time of Flight message is sent as a string.
            The publisher of this topic is 'suction_gripper.py'
            Input: @msg  type: String  (e.g. '[Ch3] Period: 57 ms, Distance: 172 mm')
        """
        
        # self.get_logger().info("TOF process callback triggered.")
        
        try:
            tof_string = msg.data
            distance_part = tof_string.split('Distance: ')[1]
            distance = int(distance_part.split(' mm')[0])
        
            # Update the distance variable          
            self.tof_moving_avg.add_value(distance)
            self.tof_distance = self.tof_moving_avg.get_average()
            # self.get_logger().info(f"Updated TOF distance: {self.tof_distance} mm")
            
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Error processing TOF message: {msg.data} | Exception: {str(e)}")


    def air_press_process(self, msg):

        try:
            self.air_pressure = msg.data
            self.air_averages = []
            for reading in self.air_pressure:
                self.air_moving_avg.add_value(reading)
                self.air_averages.append(self.air_moving_avg.get_average())

            # self.get_logger().info(f"Updated Air Pressure: {self.air_averages} hPa")

        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Error processing Air Pressure message: {msg.data} | Exception: {str(e)}")


    ## --- HANDY FUNCTIONS AND METHODS
   
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


class MovingAverage:
    def __init__(self, size=10):
        self.size = size
        self.values = deque(maxlen=10)
        self.total = 0.0

    def add_value(self, value):
        if len(self.values) == self.size:
            # Remove the oldest value from the total
            self.total -= self.values[0]
        self.values.append(value)
        self.total += value

    def get_average(self):
        if not self.values:  # Avoid division by zero
            return 0.0
        return self.total / len(self.values)
    



def main(args=None):

    rclpy.init(args=args)
    local_planner = GraspController()
    executor = MultiThreadedExecutor()
    rclpy.spin(local_planner, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
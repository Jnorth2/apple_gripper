#!/usr/bin/env python3

import numpy as np
#import pandas as pd
import scipy
from scipy.ndimage import median_filter
import os
import math
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

import tf2_geometry_msgs

# ROS2 transforms
from tf_transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

# Interfaces
from std_srvs.srv import Trigger, Empty, SetBool
from geometry_msgs.msg import Vector3, WrenchStamped, Twist, TwistStamped, TransformStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool, String, Float32MultiArray, Int16MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from gripper_msgs.srv import GripperVacuum, GripperFingers, GripperMultiplexer, SetArmGoal, GetArmPosition

# Self developed 
from air_functions import * 

class GraspController(Node):
    def __init__(self):
        super().__init__('grasp_controller')

        # Specify reentrant callback group 
        r_callback_group = ReentrantCallbackGroup()
        #-------------------------------- Node Parameters -------------------------------------#
        self.declare_parameter("gripper_type", "old")
        self.gripper_type = self.get_parameter("gripper_type").get_parameter_value().string_value

        if self.gripper_type == "finray":
            self.finger_service_type = SetBool
            self.valve_service_type = SetBool
            self.finger_service = "/microROS/actuate_odrive"
            self.suction_valve_service = "/microROS/toggle_valve"
            self.sensor_topic = "microROS/sensor_data"
            #Parameter
            self.GRIPPER_HEIGHT = 0.26456 # [m] Distance form 'tool0' to the center of the gripper with same height as engaged suctino cups
        else:
            self.finger_service_type = GripperFingers
            self.valve_service_type = GripperVacuum
            self.finger_service = "set_fingers_status"
            self.suction_valve_service = "set_vacuum_status"
            self.tof_topic = "/gripper/distance"
            self.press_topic = "/gripper/pressure"
            #Parameters
            self.GRIPPER_HEIGHT = 0.19 # [m] Distance form 'tool0' to the center of the gripper with same height as engaged suctino cups
        self.get_logger().info(f"gripper type: {self.gripper_type}")

        #----------------------------------- ROS TOPICS ---------------------------------------#
        # Publishers
        self.grasp_servo_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)        
        self.marker_text_publisher = self.create_publisher(Marker, 'caption', 10)

        # Subscribers
        if self.gripper_type == "finray":
            self.sensor_subscriber = self.create_subscription(Int16MultiArray, self.sensor_topic, self.sensor_process, 10)
        else:
            self.tof_subscriber = self.create_subscription(String, self.tof_topic, self.tof_process, 10)     
            self.air_press_subscriber = self.create_subscription(Float32MultiArray, self.press_topic, self.air_press_process, 10)
        
        # self.pose_subscription = self.create_subscription(TransformStamped,'/tool_pose', self.read_eef_pose, 10)
        

        #------------------------------------ ROS SERVICES -------------------------------------#
        # Services
        # self.text_in_rviz_srv = self.create_service(TextInRviz, 'place_text_in_rviz', self.toy_problem_callback)
        self.grasp_apple_srv = self.create_service(Trigger, 'grasp_apple', self.grasp_apple_callback, callback_group=r_callback_group)
        self.release_apple_srv = self.create_service(Trigger, 'release_apple', self.release_apple_callback, callback_group=r_callback_group)
                                  
        # Service clients
        # Theses services are providede by the node 'suction_gripper.py'
        self.vacuum_service_client = self.create_client(self.valve_service_type, self.suction_valve_service)
        while not self.vacuum_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for gripper vaccuum server")        

        self.fingers_service_client = self.create_client(self.finger_service_type, self.finger_service)
        while not self.fingers_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for gripper finger server")
        
        self.get_logger().info("All services Available!")
        
        #----------------------------------- NODE PROPERTIES ----------------------------------#
        # Recurring method
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Parameters    
        self.MAX_JOINT_SPEED = 2.0                  # Parameter to max out the joint speeds
        self.VACUUM_TRIGGER_DISTANCE = 100          # Distance to trigger vacuum during the approach
        self.ENGAGEMENT_THRESHOLD = 600             # Air pressure threshold to tell when a cup engaged
        #self.GRIPPER_HEIGHT = 0.19                  # [m] Distance form 'tool0' to the center of the gripper with same height as engaged suctino cups
        self.KP = self.MAX_JOINT_SPEED/800          # Proportional constant: converts max pressure error (1000-200 = 800)hPa to max joint speed rad/sec

        ### Tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Variables                          
        self.rate = self.create_rate(0.5)
        self.fingers_previous_action = "close"        
        # Instantiate classes        
        self.airABC_moving_avg_list = []
        for i in range(3):
            self.airABC_moving_avg_list.append(MovingAverage(10))           
        self.tof_moving_avg = MovingAverage(10)

        self.tof_distance = 200                     # units in mm             
        self.air_averages = [1000, 1000, 1000]      # units in hPa      
        self.angular_speed_x = 0.0                  # units in rad/sec
        self.angular_speed_y = 0.0                  # units in rad/sec
        self.cr_x = 0.0                             # units in m
        self.cr_y = 0.0                             # units in m
        
        # Rotation matrix 
        theta = math.radians(150)                   # angle between "gripper_scups_link" and "tool0"
        self.ROT_MATRIX = np.array([[np.cos(theta), -np.sin(theta), 0],
                                    [np.sin(theta),  np.cos(theta), 0],
                                    [0            ,  0            , 1]])
        
        # Flags
        self.running        = False
        self.move_flag      = False
        self.vacuum_flag    = False
        self.state          = "approach"
        

     ## --- ROS SERVICES CALLBACKS (SERVER) ---
    def grasp_apple_callback(self, request, response):

        self.get_logger().info("Activating grasping node")
        self.move_flag = True
        self.running = True      

        try:
            while rclpy.ok() and self.move_flag:
                # Remain in this while loop until 'self.move_flag' is set off
                self.get_logger().info(f"Grasping apple. State: \033[33m{self.state}\033[0m")   
                self.rate.sleep()                                          
                
        except Exception as e:
            self.get_logger().error(f"Error during grasping: {str(e)}")
        finally:                      

            self.move_flag = False
            self.running = False

            self.state = "deploying fingers"
            self.get_logger().info(f"Grasping apple. State: \033[33m{self.state}\033[0m")   
            self.send_fingers_request(True)

            # TODO: this is for debugging, delete when done
            time.sleep(4)    
        
        return response
    
    

    def release_apple_callback(self, request, response):
        """
        Simple service to release the apple at the end of the pick
        """

        # Open Fingers
        self.send_fingers_request(False)

        time.sleep(2)

        # Turn Vacuum Off
        self.send_vacuum_request(False)

        return response



    ## --- ROS SERVICE CLIENT CALLS (REQUESTS FROM THIS NODE) ---
    def send_vacuum_request(self, vacuum_status):
        """Function to call gripper vacuum service.
            Inputs - vacuum_status (bool): True turns the vacuum on, False turns it off
        """
        # build the request
        
        
        if self.gripper_type == "finray":
            request = SetBool.Request()
            request.data = vacuum_status
        else:
            request = GripperVacuum.Request()
            request.set_vacuum = vacuum_status
        # make the service call (asynchronously)
        self.vacuum_response = self.vacuum_service_client.call_async(request)


    def send_fingers_request(self, fingers_status):
        """Function to call gripper fingers service.
            Inputs - fingers_status (bool): True engages the fingers, False disengages
        """
        # build the request
        if self.gripper_type == "finray":
            request = SetBool.Request()
            request.data = fingers_status
        else:
            request = GripperFingers.Request()
            request.set_fingers = fingers_status
        # make the service call (asynchronously)
        self.fingers_response = self.fingers_service_client.call_async(request)
    
    
    ## --- RUNNING FUNCTION TO CONTROL THE FLOW OF THE SERVICES 
    def timer_callback(self):
        
        # Create a Twist message for moving the arm
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gripper_scups_link"
        # msg.header.frame_id = "tool0"

        if self.running:

            thr = self.ENGAGEMENT_THRESHOLD
            scA = self.air_averages[0]
            scB = self.air_averages[1]
            scC = self.air_averages[2]
            
            if self.state == "approach":
                msg.twist.linear.x = 0.0
                msg.twist.linear.y = 0.0
                msg.twist.linear.z = 0.2
                msg.twist.angular.x = 0.0
                msg.twist.angular.y = 0.0
                msg.twist.angular.z = 0.0

                if self.tof_distance < self.VACUUM_TRIGGER_DISTANCE and not self.vacuum_flag:
                    self.send_vacuum_request(True)
                    self.vacuum_flag = True

                if scA < thr or scB < thr or scC < thr:
                    self.state = "servoing"
                    msg.twist.linear.x = 0.0
                    msg.twist.linear.y = 0.0
                    msg.twist.linear.z = 0.0
                    msg.twist.angular.x = 0.0
                    msg.twist.angular.y = 0.0
                    msg.twist.angular.z = 0.0
            
            if self.state == "servoing":               

                # TRANSFORMATION CHOICE 1 (frame_id = 'tool0')              
                # 1 - Transform with rotation matrix from 'scup frame' into 'tool0 frame
                # speeds = np.array([0.1, 0.0, 0.0])
                # angular_velocities = np.dot(self.ROT_MATRIX, speeds.T)

                # TRANSFORMATION CHOICE 2 (frame_id = 'gripper_scups_link')
                # 1 - NO need to perform the rotation transformation            
                # angular_velocities = np.array([self.ref_angular_x, self.ref_angular_y, 0.0])
                angular_velocities = np.array([self.angular_speed_x, self.angular_speed_y, 0.0])

                # In both cases, translation does need to be performed to obtain linear velocities
                position_vector = np.array([-self.cr_x, -self.cr_y, -self.GRIPPER_HEIGHT])
                linear_velocities = np.cross(angular_velocities, position_vector)

                msg.twist.linear.x = linear_velocities[0]
                msg.twist.linear.y = linear_velocities[1]
                msg.twist.linear.z = linear_velocities[2]   
                msg.twist.angular.x = angular_velocities[0]  
                msg.twist.angular.y = angular_velocities[1]  
                msg.twist.angular.z = angular_velocities[2]  

            
            # If all engaged, leave!!
            if scA < thr and scB < thr and scC < thr:                
                msg.twist.linear.x = 0.0
                msg.twist.linear.y = 0.0
                msg.twist.linear.z = 0.0
                msg.twist.angular.x = 0.0
                msg.twist.angular.y = 0.0
                msg.twist.angular.z = 0.0

                self.get_logger().info("All suction cups engaged")
                self.move_flag = False
            
        else:                              
                msg.twist.linear.x = 0.0
                msg.twist.linear.y = 0.0
                msg.twist.linear.z = 0.0
                msg.twist.angular.x = 0.0
                msg.twist.angular.y = 0.0
                msg.twist.angular.z = 0.0

                self.move_flag = False       

        # Publish Twist message
        self.grasp_servo_publisher.publish(msg)


    ## --- FUNCTIONS TRIGGERED BY TOPIC SUBSCRIBERS
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
        """
        This method read the topic with the air-pressures and implements a moving average
        """

        try:
            self.air_pressure = msg.data
            air_averages = []
            for i, reading in  enumerate (self.air_pressure):                
                self.airABC_moving_avg_list[i]. add_value(reading)
                air_averages.append(self.airABC_moving_avg_list[i].get_average())            
            self.air_averages = air_averages

            self.get_logger().info("")
            self.get_logger().info(f"Updated Air Pressure: {self.air_averages} hPa")

            # Rotation axis and angle
            axis, mag = axis_angle_rotation(self.air_averages)      # Response units in [radians]
            self.get_logger().info(f"Rotation axis [deg]: {int(math.degrees(axis))}, Rotation angle [deg]: {int(math.degrees(mag))}") 

            # Center of rotation
            cr_x, cr_y = center_of_rotation(self.air_averages)      # Response units in [m]               
            self.get_logger().info(f"Center of Rotation [m]: {cr_x}, {cr_y}")    

            self.cr_x = cr_x
            self.cr_y = cr_y
            
            angle = mag * self.KP

            x_speed = angle * math.cos(axis)
            y_speed = angle * math.sin(axis)
            self.get_logger().info(f"Speeds: {round(x_speed,2)}, {round(y_speed,2)}")  

            self.angular_speed_x = x_speed
            self.angular_speed_y = y_speed

            

        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Error processing Air Pressure message: {msg.data} | Exception: {str(e)}")

    def sensor_process(self, msg):
        """Read Sensor data and implement moving averages

        This function does the same logic as the air_press_process and tof_process functions but 
        for sensor data combined in one topic. This is for esp32 grippers running microros and 
        publishing Int16MultiArray sensor data in the format [p1, p2, p3, tof]. 
        """
        try: 
            data = msg.data
            air_pressure = data[0:3]
            distance = data[-1]
            # Update the distance variable          
            self.tof_moving_avg.add_value(distance)
            self.tof_distance = self.tof_moving_avg.get_average()
            # self.get_logger().info(f"Updated TOF distance: {self.tof_distance} mm")

            #update pressure variables
            air_averages = []
            for i, reading in  enumerate (air_pressure):                
                self.airABC_moving_avg_list[i]. add_value(reading)
                air_averages.append(self.airABC_moving_avg_list[i].get_average())            
            self.air_averages = air_averages

            self.get_logger().info("")
            self.get_logger().info(f"Updated Air Pressure: {self.air_averages} hPa")

            # Rotation axis and angle
            axis, mag = axis_angle_rotation(self.air_averages)      # Response units in [radians]
            self.get_logger().info(f"Rotation axis [deg]: {int(math.degrees(axis))}, Rotation angle [deg]: {int(math.degrees(mag))}") 

            # Center of rotation
            cr_x, cr_y = center_of_rotation(self.air_averages)      # Response units in [m]               
            self.get_logger().info(f"Center of Rotation [m]: {cr_x}, {cr_y}")    

            self.cr_x = cr_x
            self.cr_y = cr_y
            
            angle = mag * self.KP

            x_speed = angle * math.cos(axis)
            y_speed = angle * math.sin(axis)
            self.get_logger().info(f"Speeds: {round(x_speed,2)}, {round(y_speed,2)}")  

            self.angular_speed_x = x_speed
            self.angular_speed_y = y_speed

            

        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Error processing Sensor Data: {msg.data} | Exception: {str(e)}")



    def read_eef_pose(self, pose_msg):
        """
        Reads pose of 'tool0' 
        """

        quat_msg = pose_msg.transform.rotation
        quat_vec = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        
        position_msg = pose_msg.transform.translation
        position_vec = [position_msg.x, position_msg.y, position_msg.z]

        position_vec[2] += self.GRIPPER_HEIGHT

        self.eef_point = position_vec
        self.eef_orientation = quat_vec


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

   
    def initial_hw_check(self):
        # Initial check of topics
        for i in range(10):
            self.get_logger().info(f"Updated TOF distance: {self.tof_distance} mm")
            self.get_logger().info(f"Updated Air Pressure: {self.air_averages} hPa")


    def quat_from_axis_angle(self, axis, angle):
        """
        Angle provided in radians
        """

        angle_rad = angle / 2
        s = math.sin(angle_rad)
        c = math.cos(angle_rad)
        q = [s * math.cos(math.radians(axis)), s * math.sin(math.radians(axis)), 0, c]

        return q
   


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
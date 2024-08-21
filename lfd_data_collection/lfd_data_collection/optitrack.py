#!/usr/bin/env python3

# This is used to playback the optitrack data. It first aligns the data by
# looking at the distance between optitrack markers on the fingers and base.
# When this distance increases, then we know the fingers are moving. It aslo
# looks through the rosbag to see when the motor is moving. It then subtracts
# these times to align the data. It then reads the data from the csv and
# publishes them as a ros pose

#Jacob Karty
#8/20/2024


#standard imports
import numpy as np
import time
import csv
from pathlib import Path
import re

#ros imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


class OptiTrackPublisher(Node):

    def __init__(self):
        super().__init__('optitrack_publisher')
        self.subscription = self.create_subscription(String, 'gripper/motor/position', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Pose, 'gripper/pose', 1)

        
        # Get the parameter value
        self.declare_parameter('arg1', 'optitrack/Take 2024-08-16 12.02.44 AM.csv')
        self.declare_parameter('arg2', 'bags/20240816_8')
        arg1 = self.get_parameter('arg1').get_parameter_value().string_value
        arg2 = self.get_parameter('arg2').get_parameter_value().string_value



        # Path to CSV file
        file_path = 'src/apple_gripper/lfd_data_collection/data/' + arg1

        everything = []
        # Read the CSV file
        with open(file_path, mode='r', newline='', encoding='utf-8') as file:
            reader = csv.reader(file)
            # Iterate over each row in the CSV
            for i, row in enumerate(reader):
                if(i>6):
                    row = [float(value) if value else np.nan for value in row]
                    everything.append(row)
        everything = np.array(everything)

        #extract data
        self.times = everything[:,1]
        ball1 = everything[:, 54:57]
        ball2 = everything[:, 57:60]
        ball3 = everything[:, 60:63]
        balls = [ball1, ball2, ball3]
        self.rigid_body_rot = everything[:, 2:6]

        #because the 5 markers are assymetric, the position that optitrack gives is not the center.
        #to calculate the center, average the 2 farthest markers that are on the base of the handle
        find_center = [everything[:, 10:13], everything[:, 14:17], everything[:, 18:21], everything[:, 22:25], everything[:, 26:29]]
        max_distance = 0
        pair = (0, 1)
        for i in range(5):
            for j in range(i + 1, 3):
                distance = np.linalg.norm(find_center[i] - find_center[j])
                if distance > max_distance:
                    max_distance = distance
                    pair = (i, j)
        rigid1 = everything[:, 4*pair[0]+10 : 4*pair[0]+13]
        rigid2 = everything[:, 4*pair[1]+10 : 4*pair[1]+13]
        self.rigid_body_pos = (rigid1 + rigid2) / 2

        typestore = get_typestore(Stores.ROS2_HUMBLE)
        initial_time = 0
        final_time = 0
        

        path = 'src/apple_gripper/lfd_data_collection/data/' + arg2

        with AnyReader([Path(path)], default_typestore=typestore) as reader:
            for connection, timestamp, rawdata in reader.messages(connections=reader.connections):
                #collect initial time
                if initial_time == 0:
                    initial_time = timestamp
                
                if connection.topic == '/gripper/motor/velocity':
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    data = re.search("(?:-*\d+)(\.*)(\d*)(?!.*\d)", msg.data)

                    if float(data.group(0)) <-10:
                        final_time = timestamp
                        break
        


        #calculate when the fingers close by finding the distance between the fingers and rigid body.
        #when the distance increases from the start position, that is the beginning of the grasp
        initial_distance = [np.linalg.norm(ball1[1,:] - self.rigid_body_pos[1, :]), np.linalg.norm(ball2[1,:] - self.rigid_body_pos[1, :]), np.linalg.norm(ball3[1,:] - self.rigid_body_pos[1, :])]
        found = False
        offset = 0

        for i in range(len(self.times)):
            for ball in balls: #nan handling
                for pos in ball[i]:
                    if np.isnan(pos):
                        continue
            for pos in self.rigid_body_pos[i]:
                if np.isnan(pos):
                    continue
            for j, ball in enumerate(balls): #calculate and compare distances
                if np.linalg.norm(ball[i,:] - self.rigid_body_pos[i, :]) > initial_distance[j]+.002:
                    offset= self.times[i] - ((final_time -initial_time)/1000000000) #difference in times
                    found = True
            if found:
                break
        

        self.get_logger().info(f"there is a {round(offset, 3)} second difference between optitrack data and ros data")
        
        self.offset = 0
        for i in range(len(self.times)):
            self.offset = i
            if self.times[i]>offset:
                break

        self.first = True



    def listener_callback(self, msg):
        if(self.first): #i only want to run this once. the listener is just for the timing
            self.get_logger().info(f"starting the playback")
            initial_time = time.time()
            self.first = False
            for i in range(self.offset, len(self.times)):
                while self.times[i] - self.times[self.offset] > time.time() - initial_time: 
                    continue #wait for the times to match
                orientation = Quaternion()
                point = Point()

                point.x = self.rigid_body_pos[i, 0]
                point.y = self.rigid_body_pos[i, 1]
                point.z = self.rigid_body_pos[i, 2]

                q = self.rigid_body_rot[i] 
                orientation.x = q[0]
                orientation.y = q[1]
                orientation.z = q[2]
                orientation.w = q[3]

                msg = Pose() #publish the pose
                msg.orientation = orientation
                msg.position = point

                self.publisher_.publish(msg)
            self.get_logger().info(f"finishing the playback")
        return

def main(args=None):
    rclpy.init(args=args)


    minimal_subscriber = OptiTrackPublisher()

    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


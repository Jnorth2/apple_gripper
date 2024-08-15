#!/usr/bin/env python3

import pandas as pd
import numpy as np
import time
import math
import csv
from pathlib import Path
import re



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


        # Path to CSV file
        file_path = 'src/apple_gripper/gripper/data/optitrack/test3.csv'
        # file_path = 'gripper/data/optitrack/test3.csv'
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

        test1 = everything[:, 10:13]
        test2 =everything[:, 26:29]
        test3 = (test1 + test2 ) / 2
        

        #psuedocode
        #run through the bag file
        #find the timestamp when the motor velocity is not zero (<-1)
        #subtract this from the initial timestamp.
        # run throught the csv file
        #find the timestamp when the distance from fingertip to base increases
        #second timestamp-first timestamp


        typestore = get_typestore(Stores.ROS2_HUMBLE)
        initial_time = 0
        final_time = 0

        with AnyReader([Path('src/apple_gripper/gripper/data/bags/LFD/20240814_4')], default_typestore=typestore) as reader:
            for connection, timestamp, rawdata in reader.messages(connections=reader.connections):
                #collect initial time
                if initial_time == 0:
                    initial_time = timestamp
                
                if connection.topic == '/gripper/motor/velocity':
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    data = re.search("(?:-*\d+)(\.*)(\d*)(?!.*\d)", msg.data)

                    # self.get_logger().info(str(float(data.group(0))))
                    if float(data.group(0)) <-1:
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
                    offset=self.times[i] - ((final_time -initial_time)/1000000000) #optitrack time minus the bag time
                    found = True
            if found:
                break
        
        self.offset = 0
        for i in range(len(self.times)):
            self.offset = i
            if self.times[i]>offset:
                break
        
        print(self.offset)

        self.first = True

                    
        # TODO - listen to a motor publishing message, align the data, and publish the pose


    def listener_callback(self, msg):
        if(self.first): #i only want to run this once. the listener is just for the timing
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

                q = self.rigid_body_rot[i] #rotate the quaternion to be in the right orientation
                # qconj = [-q[0], -q[1], -q[2], q[3]]
                # rot = (np.sin(np.pi / 8), 0, 0, np.cos(np.pi / 8))

                # q = self.quaternion_multiply(rot, self.quaternion_multiply(rot, qconj))

                orientation.x = q[0]
                orientation.y = q[1]
                orientation.z = q[2]
                orientation.w = q[3]

                msg = Pose() #publish the pose
                msg.orientation = orientation
                msg.position = point

                self.publisher_.publish(msg)
        return

    def quaternion_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2= q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        )

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












# # Read the CSV file into a DataFrame
# df = pd.read_csv('data/optitrack/test.csv')
# time = df.iloc[5:, 1]

# """
# psuedocode to find the start time
# only look at the first time step
# pick a random ball
# calculate the distance to all of the other balls
# if that has 5 low distances and 3 large ones, repeat with a different ball
# if that has 2 low distances and 6 large ones, it is on the fingertip
# the other 2 low distances are also fingertip ones
# calculate the next shortest distance for each fingertip and keep track of these 3 pairs
# if any of the distances of these three pairs changes, then the fingers have been deployed.
# """

# numballs = 3
# ball_positions = []
# for ball in range(numballs):
#     x = df.iloc[5:, 11 + 3 * ball]
#     y = df.iloc[5:, 12 + 3 * ball]
#     z = df.iloc[5:, 13 + 3 * ball]

#     position = [[float(x[i+5]), float(y[i+5]), float(z[i+5])] for i in range(len(x))]
#     ball_positions.append(position)

# average_positions = np.mean(np.array(ball_positions), axis=0)

# print(average_positions)
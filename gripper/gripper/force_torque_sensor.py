#!/usr/bin/env python3

# This file creates a ROS node to continuously read data from the serial of a
# M4313M2B 6 axis force torque sensor, and it publishes that data as a ROS topic.
#The format is a Twist, with <Fx, Fy, Fz>, <Mx, My, Mz>

#Jacob Karty
#8/2/2024

#ros imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 

#standard imports
import serial
import struct

class ForceTorque(Node):

    def __init__(self):
        #create node and publisher
        super().__init__('force_torque_publisher')
        self.publisher_ = self.create_publisher(Twist, '/gripper/force_torque', 10)


        #connect to serial
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        #request for continuous packets of data. 'AT+GOD\r\n' requests for one packet
        ser.write('AT+GSD\r\n'.encode(encoding="ascii"))

        labels = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'] #this is the format that the data is printed in

        header_check = 'aa55' #make sure that the begining of the bytes match what they should
        header = ''

        while True: #continuously read data
            while True: # read in a byte until it matches the header tag
                header += ser.read(1).hex()
                if len(header)>=4 and header[-4:]==header_check: #read in the rest of the packet after the header is matched
                    header = ''
                    break
            packet = ser.read(29).hex() # read in the rest of the packet
            data = self.parse_data(packet) # parse the data into forces
            if data != False: #if the data is read correctly
                msg = Twist()
                msg.linear.x = data[0]
                msg.linear.y = data[1]
                msg.linear.z = data[2]
                msg.angular.x = data[3]
                msg.angular.y = data[4]
                msg.angular.z = data[5]
                self.publisher_.publish(msg) #publish to topic
        

    def parse_data(self, packet):
        """
        This function takes in a packet and makes sure the check digit matches. It then parses the data and returns it.
        Input: packet of data with the header already cut off
        Output: [Fx, Fy, Fz, Mx, My, Mz] forces and torques in N and NM
        """
        sum_check = int(packet[-2:], 16) #the sum check digit 

        packet = [packet[i:i+2] for i in range(0, len(packet), 2)][4:-1] #split into pairs and remove first few and last data that we don't need
        
        #add up all of the data and compare it the the check digit
        digit_sum = sum([int(data, 16) for data in packet])
        if digit_sum%256 != sum_check:
            return False

        packet = [packet[i:i + 4] for i in range(0, len(packet), 4)] #split into the 6 channels
        packet = [''.join(sublist[::-1]) for sublist in packet] #reverse and join each channel
        packet = [struct.unpack('!f', bytes.fromhex(sublist))[0] for sublist in packet] #convert each channel using single precision floating converter
        return packet




def main(args=None):
    rclpy.init(args=args)
    FT = ForceTorque()
    rclpy.spin(FT)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
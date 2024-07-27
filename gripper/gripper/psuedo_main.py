#!/usr/bin/env python3

#imports
import os
import signal
import subprocess
import time
import datetime

#ROS imports
from gripper_msgs.srv import GripperVacuum, GripperFingers, GripperMultiplexer
import rclpy
from rclpy.node import Node



class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client')

        #set up vacuum client
        self.vacuum_client = self.create_client(GripperVacuum, 'set_vacuum_status')
        while not self.vacuum_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('vacuum service not available, waiting again...')
        self.vacuum_request = GripperVacuum.Request()
        self.get_logger().info('vacuum service connected')

        #set up fingers client
        self.fingers_client = self.create_client(GripperFingers, 'set_fingers_status')
        while not self.fingers_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('finger service not available, waiting again...')
        self.fingers_request = GripperFingers.Request()
        self.get_logger().info('fingers service connected')

        #set up multiplexer client
        self.multiplexer_client = self.create_client(GripperMultiplexer, 'set_multiplexer_status')
        while not self.multiplexer_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('multiplexer service not available, waiting again...')
        self.multiplexer_request = GripperMultiplexer.Request()
        self.get_logger().info('multiplexer service connected')

    def send_vacuum_request(self, state):
        self.vacuum_request.set_vacuum = state
        return self.vacuum_client.call_async(self.vacuum_request)
    
    def send_fingers_request(self, state):
        self.fingers_request.set_fingers = state
        return self.fingers_client.call_async(self.fingers_request)
    
    def send_multiplexer_request(self, state):
        self.multiplexer_request.set_multiplexer = state
        return self.multiplexer_client.call_async(self.multiplexer_request)


def main():
    rclpy.init()
    gripper_client = GripperClient()

    #start mux (muxOn)
    future = gripper_client.send_multiplexer_request(True)
    rclpy.spin_until_future_complete(gripper_client, future)
    response = future.result()
    gripper_client.get_logger().info(str(response))

    time.sleep(2)

    #start bag recording
    name = 'src/apple_gripper/bags/' + datetime_simplified()
    topics = ['/gripper/distance', '/gripper/pressure/sc1', '/gripper/pressure/sc2', '/gripper/pressure/sc3', '/gripper/motor/current', '/gripper/motor/position', '/gripper/motor/velocity']
    cmd = 'ros2 bag record -o ' + name + ' ' + ' '.join(topics)
    pro = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                       shell=True, preexec_fn=os.setsid) 

    time.sleep(2)

    #open valve (suctionOn)
    future = gripper_client.send_vacuum_request(True)
    rclpy.spin_until_future_complete(gripper_client, future)
    response = future.result()
    gripper_client.get_logger().info(str(response))

    time.sleep(2)

    #close fingers (fingersClose)
    future = gripper_client.send_fingers_request(True)
    rclpy.spin_until_future_complete(gripper_client, future)
    response = future.result()
    gripper_client.get_logger().info(str(response))

    time.sleep(20)

    #open fingers (fingersOpen)
    future = gripper_client.send_fingers_request(False)
    rclpy.spin_until_future_complete(gripper_client, future)
    response = future.result()
    gripper_client.get_logger().info(str(response))

    time.sleep(10)

    #close valve (suctionOff)
    future = gripper_client.send_vacuum_request(False)
    rclpy.spin_until_future_complete(gripper_client, future)
    response = future.result()
    gripper_client.get_logger().info(str(response))

    time.sleep(2)

    #stop bag recording
    os.killpg(os.getpgid(pro.pid), signal.SIGTERM)

    time.sleep(2)

    #stop mux (muxOff)
    future = gripper_client.send_multiplexer_request(False)
    rclpy.spin_until_future_complete(gripper_client, future)
    response = future.result()
    gripper_client.get_logger().info(str(response))



    gripper_client.destroy_node()
    rclpy.shutdown()



# ---------------- FILE NAMING CONVENTION ---------------------------- #
def datetime_simplified():
    """Convenient method to adapt datetime """
    year = datetime.datetime.now().year
    month = datetime.datetime.now().month
    day = datetime.datetime.now().day
    hour = datetime.datetime.now().hour
    minute = datetime.datetime.now().minute

    day = str(year) + str(month//10) + str(month%10) + str(day)
    time = str(hour//10) + str(hour%10) + str(minute//10) + str(minute%10)

    return(day)




if __name__ == '__main__':
    main()
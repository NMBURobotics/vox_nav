import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix
from ament_index_python.packages import get_package_share_directory
import os

from ruamel.yaml import YAML
import numpy as np
import sys
from threading import Thread, Lock

kFrequencyToSaveGPSPoint = 2.0


class GPSWaypoitCollector(Node):
        
    def __init__(self):
        super().__init__('gps_waypoint_collector')

        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.listener_callback, qos_profile_sensor_data
        )
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(kFrequencyToSaveGPSPoint, self.peroidic_callback)
        self.latest_navsat = NavSatFix
        self.index = 0
        self.mutex = Lock()
        self.is_first_msg_recieved = False 

    def listener_callback(self, msg):
        self.mutex.acquire()
        self.latest_navsat = msg
        self.mutex.release()
        self.is_first_msg_recieved = True 




    def peroidic_callback(self):
        if(self.is_first_msg_recieved):
            self.mutex.acquire()
            print('gps_waypoint'+str(self.index)+':', [abs(self.latest_navsat.latitude),
                                                       abs(self.latest_navsat.longitude),
                                                           self.latest_navsat.altitude])
            input("PRESS ENTER TO COLLECT NEXT WAYPOINT ")
            self.index = self.index+1
            self.mutex.release()



def main(args=None):
    rclpy.init(args=args)

    gps_waypoint_collector = GPSWaypoitCollector()

    rclpy.spin(gps_waypoint_collector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_waypoint_collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

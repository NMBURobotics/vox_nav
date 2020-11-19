import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix
from ament_index_python.packages import get_package_share_directory
import os

from ruamel.yaml import YAML
import numpy as np
import sys

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
        self.timer = self.create_timer(
            kFrequencyToSaveGPSPoint, self.peroidic_callback)
        self.latest_navsat = NavSatFix
        self.index = 0

    def listener_callback(self, msg):
        self.latest_navsat = msg

    def peroidic_callback(self):
        print('gps_waypoint'+str(self.index)+':', [self.latest_navsat.longitude,
                                                   self.latest_navsat.latitude,
                                                   self.latest_navsat.altitude])
        input("PRESS ENTER TO COLLECT NEXT WAYPOINT ")
        self.index = self.index+1


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

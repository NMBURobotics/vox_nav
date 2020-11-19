import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix


class GPSWaypoitCollector(Node):

    def __init__(self):
        super().__init__('gps_waypoint_collector')
        print("Constructing a GPSWaypoitCollector instance")

        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.listener_callback, qos_profile_sensor_data
        )
        self.subscription  # prevent unused variable warning
        print("Constructed a GPSWaypoitCollector instance")

    def listener_callback(self, msg):
        self.get_logger().info('I heard:')
        print(msg.latitude, msg.longitude, msg.altitude)


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

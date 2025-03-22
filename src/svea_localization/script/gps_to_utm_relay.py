#! /usr/bin/env python3

__author__ = "Sulthan Suresh Fazeela"
__email__ = "sultha@kth.se"
__license__ = "MIT"

import rclpy
import rclpy.exceptions
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped

import utm


class GPSToUTMRelay(Node):
    def __init__(self) -> None:
        try:
            # Initialize node
            # No more Anonymous=True
            super().__init__('gps_to_utm_relay')
            
            # Topic Parameters
            self.declare_parameter('gps_topic', 'gps/fix')
            self.gps_topic = self.get_parameter('gps_topic').value
            
            # Publishers
            self.utm_topic = self.gps_topic + '/utm'
            self.utm_pub = self.create_publisher(PointStamped, self.utm_topic, qos_profile=10)
            
            # Subscribers          
            self.gps_sub = self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, 10)
        except Exception as e:
            # Log error
            self.get_logger().error(str(e))

        else:
            # Log status
            self.get_logger().info('{} node initialized.'.format(self.get_name()))
            
    def run(self) -> None:
        try:
            rclpy.spin(self)  
        except KeyboardInterrupt:
            # Capture Ctrl+C and shutdown node
            self.get_logger().info('Shutting down {}'.format(self.get_name()))
        finally:
            # Cleanup
            self.destroy_node()
            rclpy.shutdown()

    def gps_callback(self, msg):
        easting, northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
        point = PointStamped()
        point.header = msg.header
        point.point.x = easting
        point.point.y = northing
        point.point.z = msg.altitude
        self.utm_pub.publish(point)

def main():
    rclpy.init()
    node = GPSToUTMRelay()
    node.run()
            
        
if __name__ == '__main__':
    main()
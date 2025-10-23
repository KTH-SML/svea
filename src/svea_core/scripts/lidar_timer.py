#! /usr/bin/env python3

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from rclpy.time import Duration

from svea_core import rosonic as rx

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


qos_subber = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # BEST_EFFORT
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep the last N messages
    durability=QoSDurabilityPolicy.VOLATILE,    # Volatile
    depth=10,                                   # Size of the queue
)


class lidar_filter(rx.Node):
    """
    Correcting Encoder Diraction
    """

    ## Publishers ##
    encoder_re_pub = rx.Publisher(LaserScan, '/scan/filtered', qos_pubber)

    ## Subscribers ##
    @rx.Subscriber(LaserScan, '/scan', qos_subber)
    def laser_callback(self, laser_msg):
<<<<<<< HEAD
        now = self.get_clock().now()
        adjusted_time = now - Duration(seconds=0.01)
        laser_msg.header.stamp = adjusted_time.to_msg()
=======
        now = self.get_clock().now().to_msg()
        delay_ns = int(10e6) # 10 ms
        laser_msg.header.stamp = Time(sec=now.sec, nanosec=now.nanosec - delay_ns if now.nanosec > delay_ns else 0)
>>>>>>> fad5c54 (minor fix)
        self.encoder_re_pub.publish(laser_msg)

    def on_startup(self):
        pass


if __name__ == '__main__':
    lidar_filter.main()

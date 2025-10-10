#! /usr/bin/env python3

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Bool, Int8

from svea_core import rosonic as rx

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


qos_subber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep the last N messages
    durability=QoSDurabilityPolicy.VOLATILE,    # Volatile
    depth=10,                                   # Size of the queue
)


class encoder_filter(rx.Node):
    """
    Correcting Encoder Diraction
    """

    rm_throttle = 1 
    ctrl_throttle = 1

    ## Publishers ##
    encoder_re_pub = rx.Publisher(TwistWithCovarianceStamped, '/encoder/sfiltered', qos_pubber)

    ## Subscribers ##
    @rx.Subscriber(Bool, '/lli/remote/override', qos_subber)
    def override_sub(self, override_msg):
        self.override = override_msg.data

    @rx.Subscriber(Int8, '/lli/remote/throttle', qos_subber)
    def throttle_sub(self, rm_throttle_msg):
        if rm_throttle_msg.data >= 0:
            self.rm_throttle = 1
        else:
            self.rm_throttle = -1

    @rx.Subscriber(Int8, '/lli/ctrl/throttle', qos_subber)
    def ctrl_throttle_sub(self, throttle_msg):
        if ctrl_throttle_msg.data >= 0:
            self.ctrl_throttle = 1
        else:
            self.ctrl_throttle = -1

    @rx.Subscriber(TwistWithCovarianceStamped, '/lli/sensor/encoder', qos_subber)
    def imu_sub(self, imu_msg):
        if self.override:
            imu_msg.twist.twist.linear.x = abs(imu_msg.twist.twist.linear.x) * self.rm_throttle
        else:
            imu_msg.twist.twist.linear.x = abs(imu_msg.twist.twist.linear.x) * self.ctrl_throttle
        
        self.encoder_re_pub.publish(imu_msg)

    def on_startup(self):
        pass


if __name__ == '__main__':
    encoder_filter.main()

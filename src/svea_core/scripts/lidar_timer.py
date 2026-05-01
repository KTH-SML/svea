#! /usr/bin/env python3
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from rclpy.time import Duration, Time
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
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

    from_topic = rx.Parameter('scan')
    to_topic = rx.Parameter('scan/filtered')

    target_frame = rx.Parameter('base_link')
    source_frame = rx.Parameter('odom')

    ## Publishers ##
    encoder_re_pub = rx.Publisher(LaserScan, to_topic, qos_pubber)

    ## Subscribers ##
    @rx.Subscriber(LaserScan, from_topic, qos_subber)
    def laser_callback(self, laser_msg):
        laser_msg.header.stamp = self.header.stamp
        self.encoder_re_pub.publish(laser_msg)

    def on_startup(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Use a timer to check periodically
        self.timer = self.create_timer(0.03, self.get_latest_transform_header)
        self.header = Header()
        self.header.stamp = self.get_clock().now().to_msg()

    def get_latest_transform_header(self):
        try:
            # "latest" means we don't specify a time
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.target_frame, # target
                source_frame=self.source_frame, # source
                time=Time()          # latest available
            )
            self.header = transform.header
            # self.get_logger().info(
            #     f"Latest transform header:\n"
            #     f"  Frame ID: {self.header.frame_id}\n"
            #     f"  Child Frame ID: {transform.child_frame_id}\n"
            #     f"  Stamp: {self.header.stamp.sec}.{self.header.stamp.nanosec}"
            # )

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Transform not available: {e}")


if __name__ == '__main__':
    lidar_filter.main()

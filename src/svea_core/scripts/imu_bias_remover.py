#! /usr/bin/env python3

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu

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


class imu_bias_remove(rx.Node):
    """
    Calibrating IMU bias
    """

    bias_sampled = False
    sample_count = 200
    sample_counter = 0

    ## Publishers ##
    imu_re_pub = rx.Publisher(Imu, '/imu/filtered', qos_pubber)

    ## Subscribers ##
    @rx.Subscriber(Imu, '/lli/sensor/imu', qos_subber)
    def imu_sub(self, imu_msg):
        if not self.bias_sampled:
            if self.sample_counter == 0:
                self.bias_angular_z = 0
                self.bias_linear_x = 0
                self.bias_linear_y = 0

            self.bias_angular_z = (self.bias_angular_z * self.sample_counter + imu_msg.angular_velocity.z) / (self.sample_counter + 1)
            self.bias_linear_x = (self.bias_linear_x * self.sample_counter + imu_msg.linear_acceleration.x) / (self.sample_counter + 1)
            self.bias_linear_y = (self.bias_linear_y * self.sample_counter + imu_msg.linear_acceleration.y) / (self.sample_counter + 1)

            self.sample_counter += 1

            if self.sample_counter >= self.sample_count:
                self.bias_sampled = True
                self.get_logger().info(f"IMU bias sampled: angular_z: {self.bias_angular_z}, linear_x: {self.bias_linear_x}, linear_y: {self.bias_linear_y}")
        
        else:
            imu_msg.angular_velocity.z -= self.bias_angular_z
            imu_msg.angular_velocity.z = imu_msg.angular_velocity.z / 4.0
            imu_msg.linear_acceleration.x -= self.bias_linear_x
            imu_msg.linear_acceleration.y -= self.bias_linear_y

            self.imu_re_pub.publish(imu_msg)

    def on_startup(self):
        pass


if __name__ == '__main__':
    imu_bias_remove.main()
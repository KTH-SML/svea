#! /usr/bin/env python3

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu

from svea_core import rosonic as rx

qos_pubber = QoSProfile(
<<<<<<< HEAD
<<<<<<< HEAD
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
=======
    reliability=QoSReliabilityPolicy.RELIABLE,
>>>>>>> 28b1ce8 (encoder and imu high level calibrater)
=======
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
>>>>>>> 2c22348 (minor fixs)
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


qos_subber = QoSProfile(
<<<<<<< HEAD
<<<<<<< HEAD
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # BEST_EFFORT
=======
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable
>>>>>>> 28b1ce8 (encoder and imu high level calibrater)
=======
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # BEST_EFFORT
>>>>>>> 2c22348 (minor fixs)
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep the last N messages
    durability=QoSDurabilityPolicy.VOLATILE,    # Volatile
    depth=10,                                   # Size of the queue
)


class imu_bias_remove(rx.Node):
    """
    Calibrating IMU bias
    """

    bias_sampled = False
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    sample_count = 100
=======
    sample_count = 5000
>>>>>>> 28b1ce8 (encoder and imu high level calibrater)
=======
    sample_count = 1000
>>>>>>> 2c22348 (minor fixs)
=======
    sample_count = 200
>>>>>>> 6151141 (update)
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
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 6151141 (update)
            imu_msg.angular_velocity.z = imu_msg.angular_velocity.z / 4.0
            imu_msg.linear_acceleration.x -= self.bias_linear_x
            imu_msg.linear_acceleration.y -= self.bias_linear_y

            x = imu_msg.linear_acceleration.y * -1
            y = imu_msg.linear_acceleration.x
            z = imu_msg.linear_acceleration.z * -1

            imu_msg.linear_acceleration.x = x
            imu_msg.linear_acceleration.y = y
            imu_msg.linear_acceleration.z = z

            
            if abs(imu_msg.linear_acceleration.x) < 0.05:
                imu_msg.linear_acceleration.x = 0.0

            if abs(imu_msg.angular_velocity.z) < 0.03:
                imu_msg.angular_velocity.z = 0.0

=======
            imu_msg.linear_acceleration.x -= self.bias_linear_x
            imu_msg.linear_acceleration.y -= self.bias_linear_y

>>>>>>> 28b1ce8 (encoder and imu high level calibrater)
            self.imu_re_pub.publish(imu_msg)

    def on_startup(self):
        pass


if __name__ == '__main__':
<<<<<<< HEAD
<<<<<<< HEAD
    imu_bias_remove.main()
=======
    state_publisher.main()
>>>>>>> 28b1ce8 (encoder and imu high level calibrater)
=======
    imu_bias_remove.main()
>>>>>>> 6151141 (update)

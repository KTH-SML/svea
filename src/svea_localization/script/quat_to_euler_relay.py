#! /usr/bin/env python3

__author__ = "Sulthan Suresh Fazeela"
__email__ = "sultha@kth.se"
__license__ = "MIT"

import rclpy
import rclpy.exceptions
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import math


def replace_base(old, new) -> str:
    split_last = lambda xs: (xs[:-1], xs[-1])
    is_private = new.startswith('~')
    is_global = new.startswith('/')
    assert not (is_private or is_global)
    ns, _ = split_last(old.split('/'))
    ns += new.split('/')
    return '/'.join(ns)


class QuatToEulerRelay(Node):
        
        def __init__(self) -> None:
            try:
                # Initialize node
                super().__init__('quat_to_euler_relay')
                
                # Topic Parameters
                self.quat_topic = self.load_param('~quat_topic', 'quat')
                
                # Other Parameters
                self.topic_timeout = self.load_param('~topic_timeout', 30.0)
                
                # Get quat topic type
                self.quat_topic_type, msg = self.get_topic_type(self.quat_topic, self.topic_timeout)
                if self.quat_topic_type is None:
                    raise rclpy.exceptions.RCLError(msg)
                elif self.quat_topic_type == "nav_msgs/Odometry":
                    self.quat_type = Odometry
                elif self.quat_topic_type == "sensor_msgs/Imu":
                    self.quat_type = Imu
                else:
                    raise TypeError("Invalid quaternion topic type, only Odometry and Imu msg types are supported for now.")
                
                # Publishers
                self.euler_topic = self.quat_topic + '/orientation/euler'
                self.euler_pub = self.create_publisher(Vector3Stamped, self.euler_topic, 10)

                # Subscribers
                self.quat_sub = self.create_subscription(self.quat_type, self.quat_topic, self.callback, 10)
 
            except Exception as e:
                # Log error
                self.get_logger().error(str(e))

            else:
                # Log status
                self.get_logger().info('{} node initialized.'.format(self.get_name()))       

        def load_param(self, name, value=None):
            self.declare_parameter(name, value)
            if value is None:
                assert self.has_parameter(name), f'Missing parameter "{name}"'
            return self.get_parameter(name).value
        
        def get_topic_type(self, topic: str, timeout: float = 1.0) -> str:
            # Get topic type
            time = self.get_clock().now()
            while (self.get_clock().now() - time).nanoseconds / 1e9 < timeout:
                topic_names_and_types = self.get_topic_type(topic)
                for name, types in topic_names_and_types:
                    if name == topic:
                        return types[0], ""
            return None, "Timeout while trying to get topic type for {}".format(topic)

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

        def callback(self, msg) -> None:
            try:
                # Get quaternion from message
                if self.quat_topic_type == "nav_msgs/Odometry":
                    quat = msg.pose.pose.orientation
                elif self.quat_topic_type == "sensor_msgs/Imu":
                    quat = msg.orientation
                
                # Convert quaternion to roll, pitch, and yaw
                roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
                
                # Create Vector3Stamped message (in degrees)
                euler_msg = Vector3Stamped()
                euler_msg.header = msg.header
                euler_msg.vector.x = roll * 180 / math.pi
                euler_msg.vector.y = pitch * 180 / math.pi
                euler_msg.vector.z = yaw * 180 / math.pi
                
                # Publish euler message
                self.euler_pub.publish(euler_msg)
                
            except Exception as e:
                # Log error
                self.get_logger().error(str(e))
                
def main(args=None):
    rclpy.init(args=args)
    node = QuatToEulerRelay()
    node.run()


if __name__ == '__main__':
    main()
#! /usr/bin/env python3

__author__ = "Sulthan Suresh Fazeela"
__email__ = "sultha@kth.se"
__license__ = "MIT"

import rospy
import rostopic
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import math


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)

def replace_base(old, new) -> str:
    split_last = lambda xs: (xs[:-1], xs[-1])
    is_private = new.startswith('~')
    is_global = new.startswith('/')
    assert not (is_private or is_global)
    ns, _ = split_last(old.split('/'))
    ns += new.split('/')
    return '/'.join(ns)

def get_topic_type(topic: str, timeout: float = 1.0) -> str:
    # Get topic type
    time = rospy.Time.now()
    while (rospy.Time.now() - time).to_sec() < timeout:
        topic_type, _, _ = rostopic.get_topic_type(topic)
        if topic_type:
            return topic_type, ""
    return None, "Timeout while trying to get topic type for {}".format(topic)


class QuatToEulerRelay:
        
        def __init__(self) -> None:
            try:
                # Initialize node
                rospy.init_node('quat_to_euler_relay', anonymous=True)
                
                # Topic Parameters
                self.quat_topic = load_param('~quat_topic', 'quat')
                
                # Other Parameters
                self.topic_timeout = load_param('~topic_timeout', 30.0)
                
                # Get quat topic type
                self.quat_topic_type, msg = get_topic_type(self.quat_topic, self.topic_timeout)
                if self.quat_topic_type is None:
                    raise rospy.ROSException(msg)
                elif self.quat_topic_type == "nav_msgs/Odometry":
                    self.quat_type = Odometry
                elif self.quat_topic_type == "sensor_msgs/Imu":
                    self.quat_type = Imu
                else:
                    raise TypeError("Invalid quaternion topic type, only Odometry and Imu msg types are supported for now.")
                
                # Publishers
                self.euler_topic = self.quat_topic + '/orientation/euler'
                self.euler_pub = rospy.Publisher(self.euler_topic, Vector3Stamped, queue_size=10)
                
                # Subscribers
                self.quat_sub = rospy.Subscriber(self.quat_topic, self.quat_type, self.callback)
            
            except Exception as e:
                # Log error
                rospy.logerr(e)

            else:
                # Log status
                rospy.loginfo('{} node initialized.'.format(rospy.get_name()))
                
        def run(self) -> None:
            try:
                rospy.spin()
            except rospy.ROSInterruptException:
                rospy.loginfo('Shutting down {}'.format(rospy.get_name()))
        
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
                rospy.logerr(e)
                
            

if __name__ == '__main__':
    node = QuatToEulerRelay()
    node.run()
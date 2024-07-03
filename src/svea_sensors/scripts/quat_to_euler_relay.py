#! /usr/bin/env python3

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion, Vector3

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


class QuatToEulerRelay:
        
        def __init__(self) -> None:
            try:
                # Initialize node
                rospy.init_node('quat_to_euler_relay', anonymous=True)
                
                # Topic Parameters
                self.quat_topic = load_param('~quat_topic', 'quat')
                
                self.euler_topic = load_param('~euler_topic', 'euler')
                
                # Publishers
                self.euler_pub = rospy.Publisher(self.euler_topic, Vector3, queue_size=10)
                
                # Subscribers
                self.quat_sub = rospy.Subscriber(self.quat_topic, Quaternion, self.quat_callback, queue_size=10)
            
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
        
        def quat_callback(self, msg: Quaternion) -> None:
            try:
                # Convert quaternion to roll, pitch, and yaw
                roll, pitch, yaw = euler_from_quaternion([msg.x, msg.y, msg.z, msg.w])
                
                # Create Vector3 message
                euler_msg = Vector3()
                euler_msg.x = roll
                euler_msg.y = pitch
                euler_msg.z = yaw
                
                # Publish euler message
                self.euler_pub.publish(euler_msg)
                
            except Exception as e:
                # Log error
                rospy.logerr(e)
                
            

if __name__ == '__main__':
    node = QuatToEulerRelay()
    node.run()
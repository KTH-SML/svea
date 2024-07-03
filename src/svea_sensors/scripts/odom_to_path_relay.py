#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path, PoseStamped

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


class OdomToPathRelay:
        
        def __init__(self) -> None:
            try:
                # Initialize node
                rospy.init_node('odom_to_path_relay', anonymous=True)
                
                # Topic Parameters
                self.odom_topic = load_param('~odom_topic', 'odom')
                
                self.path_topic = load_param('~path_topic', 'path')
                
                # Publishers
                self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)
                
                # Subscribers
                self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
                
                # Initialize path
                self.path = Path()
            
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
        
        def odom_callback(self, msg: Odometry) -> None:
            try:
                # Update path header
                self.path.header = msg.header
                
                # Append pose to path
                pose_stamped = PoseStamped()
                pose_stamped.header = msg.header
                pose_stamped.pose = msg.pose.pose
                self.path.poses.append(pose_stamped)
                
                # Publish path
                self.path_pub.publish(self.path)
            except Exception as e:
                # Log error
                rospy.logerr(e)
                
            

if __name__ == '__main__':
    node = OdomToPathRelay()
    node.run()
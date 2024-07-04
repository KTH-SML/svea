#! /usr/bin/env python3

import rospy
# import tf
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

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
                
                # Other parameters
                self.frame_id = load_param('~base_frame_id', 'base_link')
                
                # TF2
                # self.tf_buf = tf.Buffer()
                # self.tf_listener = tf.TransformListener(self.tf_buf)
                
                # Publishers
                self.path_topic = self.odom_topic + '/path'
                self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)
                
                # Subscribers
                self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
                
                # Initialize path
                self.path = Path()
            
            except Exception as e:
                # Log error
                rospy.logerr(e)

            else:
                # Sleep for 1 second for tf2 buffer to fill
                rospy.sleep(1)
                
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
                
                # Transform pose to base frame
                pose_stamped = PoseStamped()
                pose_stamped.header = msg.header
                pose_stamped.pose = msg.pose.pose
                # try:
                #     pose_stamped = self.tf_listener.transformPose(self.frame_id, pose_stamped)
                # except Exception as e:
                #     # Log error
                #     rospy.logerr("{}: {}".format(rospy.get_name(), e))
                #     return                   
                    
                # Append pose to path
                self.path.poses.append(pose_stamped)
                
                # Publish path
                self.path_pub.publish(self.path)
            except Exception as e:
                # Log error
                rospy.logerr(e)
                
            

if __name__ == '__main__':
    node = OdomToPathRelay()
    node.run()
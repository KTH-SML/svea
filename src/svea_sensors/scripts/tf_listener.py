#!/usr/bin/env python  
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node('odom_gps_pub')
    pub = rospy.Publisher('/odometry/gps', Odometry, queue_size=1, latch=True)
    while True:
        msg = Odometry()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        quat = quaternion_from_euler(0,0,0)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.x = quat[1]
        msg.pose.pose.orientation.x = quat[2]
        msg.pose.pose.orientation.x = quat[3]
        pub.publish(msg)
        rospy.sleep(1)

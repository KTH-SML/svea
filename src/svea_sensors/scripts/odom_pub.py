#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

class Republish():

    def __init__(self):
        # Establish publisher of converted Twist message
        self.odom_pub = rospy.Publisher(
            '/odometry/gps',
            Odometry,
            queue_size=1,
            latch=True)

    def pub(self):
        odom_corr = Odometry()
        cov_matrix = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        odom_corr.header.stamp = rospy.Time.now()
        odom_corr.header.frame_id = 'odom'
        odom_corr.child_frame_id = 'base_link'
        odom_corr.pose.covariance = cov_matrix
        odom_corr.twist.covariance = cov_matrix
        self.odom_pub.publish(odom_corr)

if __name__ == '__main__':
    rospy.init_node('odom_corrector', anonymous=False)
    ctt = Republish()
    rospy.loginfo("odom_corrector node successfuly initilized")
    while not rospy.is_shutdown():
        ctt.pub()
        rospy.sleep(1)
    rospy.spin()

#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped

class Republish():

    def __init__(self):
        # Establish publisher of converted Twist message
        self.odom_pub = rospy.Publisher('/odometry/gps', Odometry, queue_size=1, latch=True)
        self.twist_pub = rospy.Publisher('/ekf_twist', TwistWithCovarianceStamped, queue_size=1, latch=True)

    def pub_odom_message(self):
        odom_corr = Odometry()
        cov_matrix = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        odom_corr.header.stamp = rospy.Time.now()
        odom_corr.header.frame_id = 'map'
        odom_corr.child_frame_id = 'base_link'
        odom_corr.pose.covariance = cov_matrix
        odom_corr.twist.covariance = cov_matrix
        self.odom_pub.publish(odom_corr)

    def pub_twist_message(self):
        twist = TwistWithCovarianceStamped()
        cov_matrix = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = 'base_link'
        twist.twist.twist.linear.x = 0.0
        twist.twist.covariance = cov_matrix
        self.twist_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('odom_corrector', anonymous=False)
    ctt = Republish()
    rospy.loginfo("odom_corrector node successfuly initilized")
    while not rospy.is_shutdown():
        #ctt.pub_odom_message()
        ctt.pub_twist_message()
        rospy.sleep(.1)
    rospy.spin()

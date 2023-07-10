#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

class Republish():

    def __init__(self):
        # Establish subscription to control message
        rospy.Subscriber('/odometry/gps', Odometry, self.odom_callback)
        # Establish publisher of converted Twist message
        self.odom_pub = rospy.Publisher(
            '/odometry/gps/corrected',
            Odometry,
            queue_size=1,
            latch=True)

    def odom_callback(self, msg):
        odom_corr = Odometry()
        odom_corr = msg
        odom_corr.header.frame_id = 'map'
        odom_corr.child_frame_id = 'base_link'
        self.odom_pub.publish(odom_corr)

if __name__ == '__main__':
    rospy.init_node('odom_corrector', anonymous=False)
    ctt = Republish()
    rospy.loginfo("odom_corrector node successfuly initilized")
    rospy.spin()

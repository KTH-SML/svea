#! /usr/bin/env python3

import numpy as np

import rospy
import tf2_ros
import tf_conversions
import message_filters as mf
from std_msgs.msg import Int16

class test_wp3go:

    def __init__(self):

        ## Initialize node
        rospy.init_node('test_wp3go')

        ## Publishers
        self.pub_wp3go = rospy.Publisher('test_wp3go', Int16, queue_size=5)
        self.count = 0
    def run(self):
        msg = Int16()
        while not rospy.is_shutdown():
            msg.data = self.count%11
            self.pub_wp3go.publish(msg)
            self.count += 1
            rospy.sleep(1)
        rospy.spin()

if __name__ == '__main__':
    test_wp3go().run()
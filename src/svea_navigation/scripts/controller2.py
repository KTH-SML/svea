#!/usr/bin/env python3

import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist

class Controller2(object):

    def __init__(self, vehicle_name=''):
        rospy.Subscriber("/cmd_vel", Twist, self.blp_control, queue_size=1)
        self.vehicle_name = vehicle_name
        self.steering = 0.0
        self.velocity = 0.0

    def blp_control(self, data):
        self.steering = data.angular.z*1.3
        self.velocity = data.linear.x

        # if self.velocity > 0 and self.velocity < 0.3:
        #     self.velocity = 0.3
        # elif self.velocity < 0 and self.velocity > -0.3:
        #     self.velocity = -0.3


    def compute_control(self, state):
        return self.steering, self.velocity


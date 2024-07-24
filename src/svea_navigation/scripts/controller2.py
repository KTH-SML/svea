#!/usr/bin/env python3

import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist

class Controller2(object):

    MIN_SPEED = 0.2
    DEADZONE = 0 # 0.07

    def __init__(self, vehicle_name=''):
        rospy.Subscriber("/cmd_vel", Twist, self.blp_control, queue_size=1)
        self.vehicle_name = vehicle_name
        self.steering = 0.0
        self.velocity = 0.0

    def blp_control(self, data):
        raw_steering = data.angular.z 
        raw_velocity = data.linear.x

        if abs(raw_velocity) <= self.DEADZONE:
            self.velocity = 0.0
        elif raw_velocity > self.DEADZONE and raw_velocity < self.MIN_SPEED:
            self.velocity = self.MIN_SPEED
        elif raw_velocity < -self.DEADZONE and raw_velocity > -self.MIN_SPEED:
            self.velocity = -self.MIN_SPEED
        else:
            self.velocity = raw_velocity

        self.steering = raw_steering * 1.5


    def compute_control(self, state):
        return self.steering, self.velocity


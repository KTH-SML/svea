#!/usr/bin/env python3

import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist

class BaseLocalPlannerController(object):

    MIN_SPEED = 0.3
    MIN_STEER = 0.2
    DEADZONE_VEL = 0.0
    DEADZONE_STEER = 0.1
    MAX_STEERING_CHANGE = 0.2  # Maximum change in steering per time step. (0.2 = 12 deg. SVEA has 120 deg. steering window.)


    def __init__(self, vehicle_name=''):
        rospy.Subscriber("/cmd_vel", Twist, self.blp_control, queue_size=1)
        self.vehicle_name = vehicle_name
        self.steering = 0.0
        self.velocity = 0.0
        self.last_steering = 0.0

    def blp_control(self, data):
        raw_steering = data.angular.z
        raw_velocity = data.linear.x

        # Handle velocity with deadzone and minimum speed
        if abs(raw_velocity) <= self.DEADZONE_VEL:
            self.velocity = 0.0
        elif raw_velocity > self.DEADZONE_VEL and raw_velocity < self.MIN_SPEED:
            self.velocity = self.MIN_SPEED
        elif raw_velocity < -self.DEADZONE_VEL and raw_velocity > -self.MIN_SPEED:
            self.velocity = -self.MIN_SPEED
        else:
            self.velocity = raw_velocity

        # Calculate the new steering value
        target_steering = raw_steering * 1.5

        # # Implements minimum steering logic with deadzone
        # if abs(target_steering) <= self.DEADZONE_STEER:
        #     target_steering = 0.0
        # elif target_steering > self.DEADZONE_STEER and target_steering <= self.MIN_STEER:
        #     target_steering = self.MIN_STEER
        # elif target_steering < -self.DEADZONE_STEER and target_steering >= -self.MIN_STEER:
        #     target_steering = -self.MIN_STEER
        # else:
        #     target_steering = target_steering

        # Apply rate of change limit to steering
        steering_change = target_steering - self.last_steering
        if abs(steering_change) > self.MAX_STEERING_CHANGE:
            # Limit the rate of change
            steering_change = self.MAX_STEERING_CHANGE * (steering_change / abs(steering_change))

        self.steering = self.last_steering + steering_change

        # Update the last steering value
        self.last_steering = self.steering


    def compute_control(self, state):
        return self.steering, self.velocity


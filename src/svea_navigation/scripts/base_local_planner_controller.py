#!/usr/bin/env python3
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
class BaseLocalPlannerController(object):
    MIN_SPEED = 0.25
    MIN_STEER = 0.2
    MAX_STEERING_CHANGE = 0.1  # Maximum change in steering per time step. (0.2 = 12 deg. SVEA has 120 deg. steering window.)

    RAW_STEERING_AMPLIFICATION = 2

    def __init__(self, vehicle_name=''):
        rospy.Subscriber("/cmd_vel", Twist, self.blp_control, queue_size=1)
        self.vehicle_name = vehicle_name
        self.steering = 0.0
        self.velocity = 0.0
        self.last_steering = 0.0
        self.last_velocity = 0.0
        self.n = 3  # Number of consecutive callbacks to publish zero velocity when a change of direction is detected. this is preventing high oscillations.
        self.direction_change_count = 0
    def blp_control(self, data):
        raw_steering = data.angular.z
        raw_velocity = data.linear.x
        # Check if the sign of velocity has changed
        direction_changed = (self.last_velocity > 0 and raw_velocity < 0) or (self.last_velocity < 0 and raw_velocity > 0)
        if direction_changed:
            # Reset the direction change counter
            self.direction_change_count = self.n
            self.velocity = 0
        else:
            # If direction change counter is greater than 0, decrement it and publish zero velocity.
            if self.direction_change_count > 0:
                self.direction_change_count -= 1
                self.velocity = 0
            else:
                # Make sure velocity is above the minimum threshold 
                self.velocity = self.velocity_lower_sat(raw_velocity)

        # Calculate the new steering value
        target_steering = raw_steering * 2
        target_steering = raw_steering * self.RAW_STEERING_AMPLIFICATION
        # Apply rate of change limit to steering
        steering_change = target_steering - self.last_steering
        if abs(steering_change) > self.MAX_STEERING_CHANGE:
            steering_change = self.MAX_STEERING_CHANGE * (steering_change / abs(steering_change))
        self.steering = self.last_steering + steering_change
        # Update the last values
        self.last_steering = self.steering
        self.last_velocity = self.velocity
    def compute_control(self, state):
        return self.steering, self.velocity
    def velocity_lower_sat(self, velocity):
        if velocity > 0:
            velocity = max(velocity, self.MIN_SPEED)
        elif velocity < 0:
            velocity = min(velocity, -self.MIN_SPEED)
        else:
            velocity = 0
        return velocity
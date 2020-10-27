#!/usr/bin/env python  
import sys
import os
import math
import rospy
import numpy as np

from svea_msgs.msg import lli_ctrl
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from svea.states import SVEAControlValues


class Republish():

    # assumed max velocity
    # By testing, the max velocity in Gear 0 is around 1.7 m/s.
    # The max velocity in Gear 1 is around 3.6 m/s.
    MAX_SPEED_0 = 1.7 # [m/s]
    MAX_SPEED_1 = 3.6 # [m/s]
    MAX_STEERING_ANGLE = 40*math.pi/180
    VELOCITY_DEAD_ZONE = 15
    TAU0 = 0.1
    TAU1 = 0.4

    def __init__(self):
        ## Pull necessary ROS parameters from launch file:
        # Read control message topic
        self.ctrl_msg_top = rospy.get_param("~ctrl_message_topic", "lli/ctrl_actuated")
        # Read twist message topic
        self.twist_msg_top = rospy.get_param("~twist_message_topic", "actuation_twist")
        # Read vehicle frame id topic
        self.vehicle_frame_id = rospy.get_param("~twist_message_topic", "base_link")
        # Read max speed
        self.max_speed = rospy.get_param("~max_speed", 0.1)
        # Read covariance values
        self.lin_cov = rospy.get_param("~linear_covariance", 0.1)
        self.ang_cov = rospy.get_param("~angular_covariance", 0.1)
        self.rate = rospy.get_param("~rate", 50)
        # Acceleration coefficient for gear 0 and gear 1
        self.tau0 = rospy.get_param("~tau0", self.TAU0)
        self.tau1 = rospy.get_param("~tau1", self.TAU1)
        
        # Initialize class variables
        self.twist_msg = TwistWithCovarianceStamped()
        self.twist_msg.header.frame_id = self.vehicle_frame_id
        self.twist_msg.twist.covariance = self.cov_matrix_build()
        self._is_reverse = False
        self._last_calc_time = None
        self._actuation_values = SVEAControlValues()
        self.velocity = 0.0
        # Establish subscription to control message
        rospy.Subscriber(self.ctrl_msg_top, lli_ctrl, self.ctrl_msg_callback)
        # Establish publisher of converted Twist message
        self.twist_pub = rospy.Publisher(self.twist_msg_top, TwistWithCovarianceStamped, queue_size=10)


    def ctrl_calc_and_pub(self):
        # initialize message
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # Likely need to change this to contunially publish commands when not on remote control
            # RC constantly publishes controls, while computer would send command until it needs to be changed
            if self._actuation_values.gear is not None:
                c_ang = self._steer_actuation_to_rad(self._actuation_values.steering)
                # Apply Bicycyle Model
                wheelbase = .32 # in meters
                vel = self.calc_current_velocity()
                B = math.atan2(math.tan(c_ang),2)
                ang_vel = (vel/(wheelbase/2))*math.sin(B)

                # Build Header for current time stamp
                self.twist_msg.header.seq += 1
                self.twist_msg.header.stamp = rospy.Time.now()

                # Build Twist using bicycle model
                self.twist_msg.twist.twist.linear.x = vel
                #self.twist_msg.twist.twist.linear.y = y_vel
                self.twist_msg.twist.twist.angular.z = ang_vel

                # Publish message
                self.twist_pub.publish(self.twist_msg)
                rate.sleep()
        rospy.spin()

    def cov_matrix_build(self):
        self.cov_matrix = [self.lin_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, self.lin_cov, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, self.lin_cov, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, self.ang_cov, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, self.ang_cov, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, self.ang_cov]
        return self.cov_matrix

    # Callback function for fiducial pose subscription (from aruco_detect)
    def ctrl_msg_callback(self, ctrl_msg):
        self._is_reverse = self._detect_reverse_state(ctrl_msg)
        self.velocity = self.calc_current_velocity() 
        self._actuation_values.ctrl_msg = ctrl_msg
        
    def _detect_reverse_state(self, msg):
        dead_zone = 5 # velocities with abs values <= than this = 0 to ESC
        velocity = msg.velocity
        previous_velocity = self._actuation_values.velocity
        if velocity > dead_zone or previous_velocity == -128:
            is_reverse = False
            if self._is_reverse:
                rospy.logdebug("is_reverse changed to False, velocity: {},\tprevious_velocity: {}".format(velocity, previous_velocity))
        elif (previous_velocity < -dead_zone and
              abs(velocity) <= dead_zone):
            is_reverse = True
            if not self._is_reverse:
                rospy.logdebug("is_reverse changed to True, velocity: {},\tprevious_velocity: {}".format(velocity, previous_velocity))
        else:
            is_reverse = self._is_reverse
        return is_reverse
    
    def calc_current_velocity(self):
        act_values = self._actuation_values
        time_now = rospy.Time.now()
        if self._last_calc_time is not None:
            dt = (time_now - self._last_calc_time).to_sec()
        else:
            dt = 0.1
        self._last_calc_time = time_now
        if (abs(act_values.velocity) < act_values.valid_range):
            setpoint_velocity = self._vel_actuation_to_mps(act_values.velocity)
            acc = self._sim_esc(self.velocity, setpoint_velocity)
            self.velocity += acc * dt
            if self.velocity < 0.0 and not self._is_reverse:
                self.velocity = 0.0
        return self.velocity
    
    def _sim_esc(self, velocity, target_velocity):
        # simulates esc dynamics
        tau = self.tau1 if self._actuation_values.velocity else self.tau0
        return 1/tau * (target_velocity - velocity)

    def _steer_actuation_to_rad(self, steering):
        """Convert steering actuation value to radians"""
        steering = float(steering)
        steer_percent = steering/127.0 * self.MAX_STEERING_ANGLE
        steer_percent = -steer_percent  # steering flipped
        return steer_percent

    def _vel_actuation_to_mps(self, vel_actuation):
        if abs(vel_actuation) < self.VELOCITY_DEAD_ZONE:
            max_speed = 0
        elif self._actuation_values.gear == 0:
            max_speed = self.MAX_SPEED_0 / 127.0
        elif self._actuation_values.gear == 1:
            max_speed = self.MAX_SPEED_1 / 127.0
        return vel_actuation * max_speed


if __name__ == '__main__':

    rospy.init_node('actuation_to_twist', anonymous=False)
    ctt = Republish()
    ctt.ctrl_calc_and_pub()
    rospy.loginfo("actuation_to_twist node successfuly initilized")



#!/usr/bin/env python

"""
Module containing teleoperation interfaces for the RC remote
"""

import math
from threading import Thread
from collections import deque
import rospy

from svea_msgs.msg import lli_ctrl
from svea.states import SVEAControlValues

__license__ = "MIT"
__maintainer__ = "Frank Jiang"
__email__ = "frankji@kth.se "
__status__ = "Development"


class RCInterface(object):
    """Interface handling the reception of RC inputs that are
    published by the low-level interface.

    :param vehicle_name: Name of vehicle being controlled;
                         The name will be effectively be added as a
                         namespace to the topics used by the
                         corresponding localization node i.e
                         `namespace/vehicle_name/state`, defaults to
                         ''
    :type vehicle_name: str, optional
    """

    MAX_SPEED_0 = 1.7 # [m/s]
    MAX_SPEED_1 = 3.6 # [m/s]
    MAX_STEERING_ANGLE = 40*math.pi/180
    PERC_TO_LLI_COEFF = 1.27
    VEL_DEADBAND_MAG = 0.05

    def __init__(self, vehicle_name='', log_length=100):
        sub_namespace = vehicle_name + '/' if vehicle_name else ''
        self._rc_topic = '{}lli/remote'.format(sub_namespace)
        if vehicle_name:
            self.vehicle_name = vehicle_name
        else:
            namespace = rospy.get_namespace()
            self.vehicle_name = namespace.split('/')[-2]

        self._ctrl_values = SVEAControlValues(0, 0, 0, False, False)
        # note, this is not the same as the data handler log
        self.rc_log = deque(maxlen=log_length)

    def start(self):
        """Spins up ROS background thread; must be called to start
        receiving data

        :return: itself
        :rtype: RCInterface
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting RC Interface Node for "
                      + self.vehicle_name)
        self.node_name = 'rc_node'
        self._start_listen()
        rospy.spin()

    def _start_listen(self):
        rospy.Subscriber(self._rc_topic,
                         lli_ctrl,
                         self._read_rc,
                         tcp_nodelay=True)

    def _read_rc(self, msg):
        # update object's log
        self.rc_log.append(msg)
        self._ctrl_values.update_from_msg(msg)

    def _percent_to_steer(self, steer_percent):
        steer_percent = float(steer_percent)
        steering = steer_percent/100*self.MAX_STEERING_ANGLE
        return steering

    def _percent_to_vel(self, vel_percent):
        vel_percent = float(vel_percent)
        if self._ctrl_values.gear == 0:
            velocity = vel_percent*self.MAX_SPEED_0 / 100
        elif self._ctrl_values.gear == 1:
            velocity = vel_percent*self.MAX_SPEED_1 / 100
        if abs(velocity) < self.VEL_DEADBAND_MAG:
            velocity = 0.0
        return velocity

    @property
    def steering(self):
        steer_percent = self._ctrl_values.steering / self.PERC_TO_LLI_COEFF
        return self._percent_to_steer(steer_percent)

    @property
    def velocity(self):
        vel_percent = self._ctrl_values.velocity / self.PERC_TO_LLI_COEFF
        return self._percent_to_vel(vel_percent)

    @property
    def gear(self):
        return self._ctrl_values.gear
    @property
    def gear_str(self):
        return "Low" if self.gear == 0 else "High" if self.gear == 1 else 'DNC'

    @property
    def front_diff_locked(self):
        return self._ctrl_values.front_diff_locked
    @property
    def front_diff_locked_str(self):
        return self._ctrl_values._diff_status_as_text(self.front_diff_locked)

    @property
    def rear_diff_locked(self):
        return self._ctrl_values.rear_diff_locked
    @property
    def rear_diff_locked_str(self):
        return self._ctrl_values._diff_status_as_text(self.rear_diff_locked)

    @property
    def control_flags(self):
        return self._ctrl_values.control_flags
    @property
    def control_flags_str(self):
        return '0b' + '0'*(10-len(bin(self.control_flags))) + bin(self.control_flags)[2:]

    def _build_param_printout(self):
        # collect important params
        steering = self.steering
        velocity = self.velocity
        gear = self.gear_str
        front_diff = self.front_diff_locked_str
        rear_diff = self.rear_diff_locked_str
        flags = self.control_flags_str

        return  ("\n## RC Input: {0}\n".format(self.vehicle_name)
                +"      steering[deg] - {0}\n".format(math.degrees(steering))
                +"      velocity[m/s] - {0}\n".format(velocity)
                +"      gear          - {0}\n".format(gear)
                +"      diff_front  - {0}\n".format(front_diff)
                +"      diff_rear   - {0}\n".format(rear_diff)
                +"      ctrl_flags  - {0}\n".format(flags))
        # return str(self._ctrl_values)

    def __repr__(self):
        return self._build_param_printout()

    def __str__(self):
        return self._build_param_printout()

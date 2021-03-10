#!/usr/bin/env python

"""
Module for bicycle models with params set for SVEA cars.
"""

import numpy as np
import math
import rospy
from svea.states import VehicleState

__license__ = "MIT"
__maintainer__ = "Frank Jiang"
__email__ = "frankji@kth.se"
__status__ = "Development"


class SimpleBicycleModel(object):
    """ Simple Bicycle Model
    State-space equation is:

        .. math::

            \\dot{x}     &= v\\cos(\\phi), \\\\
            \\dot{y}     &= v\\sin(\\phi), \\\\
            \\dot{\\phi}  &= \\frac{v}{L} \\tan(\\delta), \\\\
            \\dot{v}     &= a.

    where :math:`x, y, \phi, v` are x position, y position, yaw,
    and velocity; :math:`\delta, a` are the steering angle and
    acceleration inputs; and :math:`L` is the wheel
    base length. This object also includes a method for dynamics updates
    based on the set sampling time and the embedded bicycle model.
    Units are [m, rad, s, m/s]

    :param state: Initial state of model, defaults to origin
    :type state: VehicleState
    """

    L = 0.32
    DELTA_MAX = 40*math.pi/180  # max steering angle [rad]

    TAU = 0.1 # gain for simulating SVEA's ESC

    def __init__(self, state=VehicleState()):
        """ Initialize state. """
        self.state = state
        self.steering = 0
        self.state.time_stamp = rospy.get_rostime()

    def __repr__(self):
        return self.state.__repr__()

    def __str__(self):
        return self.state.__str__()

    def _sim_esc(self, velocity, target_velocity):
        # simulates esc dynamics
        return 1/self.TAU * (target_velocity - velocity)

    def _update(self, state, accel, delta, dt):
        # update state using simple bicycle model dynamics
        delta = np.clip(delta, -self.DELTA_MAX, self.DELTA_MAX)
        x, y, yaw, v = state
        state.x += v * np.cos(yaw) * dt
        state.y += v * np.sin(yaw) * dt
        state.yaw += v / self.L * np.tan(delta) * dt
        state.v += accel * dt

    def update(self, steering, velocity, dt):
        """
        Updates state using set sampling time, dt, and embedded bicycle
        dynamics. Designed to take same inputs as SVEA vehicle's
        low-level interface.

        :param steering: input steering angle for the car
        :type steering: float
        :param velocity: input velocity for the car
        :type velocity: float
        """
        accel = self._sim_esc(self.state.v, velocity)
        delta = steering
        self.steering = steering
        self._update(self.state, accel, delta, dt)
        self.state.time_stamp += rospy.Duration.from_sec(dt)

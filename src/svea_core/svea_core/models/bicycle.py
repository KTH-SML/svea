"""
Author: Frank Jiang
"""
import numpy as np
import math
from typing import Optional

import rclpy
import rclpy.clock
import rclpy.duration
from rclpy.clock import ClockType
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
import tf_transformations as tf

class SimpleBicycleModel:
    r"""Simple Bicycle Model.

    State-space equation is:

    $$
    \begin{aligned}
        \dot{x}     &= v\cos(\phi),                 \\
        \dot{y}     &= v\sin(\phi),                 \\
        \dot{\phi}  &= \frac{v}{L} \tan(\delta),    \\
        \dot{v}     &= a.
    \end{aligned}
    $$

    where $x, y, \phi, v$ are $x$ position, $y$ position, yaw, and velocity;
    $\delta, a$ are the steering angle and acceleration inputs; and $L$ is the
    wheel base length. This object also includes a method for dynamics updates
    based on the set sampling time and the embedded bicycle model. Units are
    `[m, rad, s, m/s]`.

    Args:
        state: Initial state of model, defaults to origin.
    """

    L = 0.32
    DELTA_MAX = 40*math.pi/180  # max steering angle [rad]

    TAU = 0.1 # gain for simulating SVEA's ESC

    def __init__(self):
        self.state = Odometry()
        self.steering = 0
        self.state.header.stamp = rclpy.clock.Clock(clock_type=ClockType.ROS_TIME).now().to_msg()
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
        x = state.pose.pose.position.x
        y = state.pose.pose.position.y
        yaw = tf.euler_from_quaternion([
            state.pose.pose.orientation.x,
            state.pose.pose.orientation.y,
            state.pose.pose.orientation.z,
            state.pose.pose.orientation.w])[2]
        v = state.twist.twist.linear.x
        # update state
        self.state.pose.pose.position.x = x + v * np.cos(yaw) * dt
        self.state.pose.pose.position.y = y + v * np.sin(yaw) * dt
        self.state.pose.pose.orientation = tf.quaternion_from_euler(
            yaw + v / self.L * np.tan(delta) * dt)
        self.state.twist.twist.linear.x = v + accel * dt

    def update(self, steering: float, velocity: float, dt: float):
        """Updates state.

        Updates state using set sampling time, `dt`, and embedded bicycle
        dynamics. Designed to take same inputs as SVEA vehicle's low-level
        interface.

        Args:
            steering: Input steering angle for the car
            velocity: Input velocity for the car
        """
        accel = self._sim_esc(self.state.v, velocity)
        delta = steering
        self.steering = steering
        self._update(self.state, accel, delta, dt)
        self.state.header.stamp = self.add_time(rclpy.duration.Duration(seconds=dt).to_msg())

    def add_time(self, delta_time: Time) -> Time:
        """Update time to the state.

        Args:
            delta_time: Time to add to the state
        """
        total_sec = self.state.header.stamp.sec + delta_time.sec
        total_nanosec = self.state.header.stamp.nanosec + delta_time.nanosec
        if total_nanosec >= 1e9:
            total_sec += 1
            total_nanosec -= 1e9
        return Time(sec=total_sec, nanosec=total_nanosec)
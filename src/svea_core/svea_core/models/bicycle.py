<<<<<<< HEAD
<<<<<<< HEAD
"""
Author: Frank Jiang
"""
import numpy as np
import math

from rclpy.clock import ClockType
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
import tf_transformations as tf

__all__ = [
    'Bicycle4D',
    'Bicycle4DWithESC',
]

class Bicycle4D:
=======
=======
"""
Author: Frank Jiang
"""
>>>>>>> 54289ac (2025/04/16 Meeting Update)
import numpy as np
import math
from typing import Optional
from abc import ABC, abstractmethod

import rclpy
import rclpy.clock
import rclpy.duration
from rclpy.clock import ClockType
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
import tf_transformations as tf

class SimpleBicycleModel:
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)
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
<<<<<<< HEAD
        initial_state: Initial state of model, defaults to origin.
    """

    L = 0.32
    DELTA_MAX = 40 * (math.pi/180)  # max steering angle [rad]
    ACCEL_MAX = 2.                  # max acceleration [m/s]

    def __init__(self, initial_state=(0., 0., 0., 0.), dt=0.1):
        self.state = initial_state
        self.dt = dt

    x = property(lambda self: self.state[0])
    y = property(lambda self: self.state[1])
    yaw = property(lambda self: self.state[2])
    vel = property(lambda self: self.state[3])

    def update(self, delta: float, accel: float, dt=None):

        if dt is None:
            dt = self.dt

        # Input
        delta = np.clip(delta, -self.DELTA_MAX, self.DELTA_MAX)
        accel = np.clip(accel, -self.ACCEL_MAX, self.ACCEL_MAX)

        # State        
        x, y, yaw, vel = self.state

        # Update
        x += vel * np.cos(yaw) * dt
        y += vel * np.sin(yaw) * dt
        yaw += vel / self.L * np.tan(delta) * dt
        vel += accel * dt
        self.state = (x, y, yaw, vel)

        return self.state

class Bicycle4DWithESC(Bicycle4D):

    TAU = 0.1 # gain for simulating SVEA's ESC

    def update(self, steering: float, velocity: float, **kwds):
        """Updates state.
        
        Designed to take same inputs as SVEA vehicle's low-level interface.
=======
        state: Initial state of model, defaults to origin.
    """

    L = 0.32
    DELTA_MAX = 40*math.pi/180  # max steering angle [rad]

    TAU = 0.1 # gain for simulating SVEA's ESC

    def __init__(self, state: Optional[Odometry] = None):
        self.state = Odometry() if state is None else state
        self.state.header.stamp = rclpy.clock.Clock(clock_type=ClockType.ROS_TIME).now().to_msg()

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
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)

        Args:
            steering: Input steering angle for the car
            velocity: Input velocity for the car
        """
<<<<<<< HEAD

        # With ESC dynamics
        delta = steering
        accel = 1/self.TAU * (velocity - self.vel)

        return super().update(delta, accel, **kwds)
=======
        accel = self._sim_esc(self.state.v, velocity)
        delta = steering
        self._update(self.state, accel, delta, dt)
<<<<<<< HEAD
        self.state.time_stamp += rclpy.duration.Duration(seconds=dt)
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)
=======
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
>>>>>>> 7ecd314 (20250428 update)

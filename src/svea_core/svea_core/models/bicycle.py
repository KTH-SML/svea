<<<<<<< HEAD
<<<<<<< HEAD
"""
Author: Frank Jiang
"""
import numpy as np
import math
<<<<<<< HEAD

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
=======
>>>>>>> ecbaf1d (Simulation and simulated lidar workes, 3d model display in vizulization)

from rclpy.clock import ClockType
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
import tf_transformations as tf

<<<<<<< HEAD
class SimpleBicycleModel:
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)
=======
__all__ = [
    'Bicycle4D',
    'Bicycle4DWithESC',
]

class Bicycle4D:
>>>>>>> 4b0286b (More work on simulator)
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
<<<<<<< HEAD
        self.state = (x, y, yaw, vel)

        return self.state

class Bicycle4DWithESC(Bicycle4D):

    TAU = 0.1 # gain for simulating SVEA's ESC

    def update(self, steering: float, velocity: float, **kwds):
        """Updates state.
        
        Designed to take same inputs as SVEA vehicle's low-level interface.
=======
        state: Initial state of model, defaults to origin.
=======
        initial_state: Initial state of model, defaults to origin.
>>>>>>> 4b0286b (More work on simulator)
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
        x += vel * np.cos(yaw) * self.dt
        y += vel * np.sin(yaw) * self.dt
        yaw += vel / self.L * np.tan(delta) * self.dt
        vel += accel * self.dt
=======
>>>>>>> 217dc92 (05/12/2025 meeting update)
        self.state = (x, y, yaw, vel)

        return self.state

class Bicycle4DWithESC(Bicycle4D):

    TAU = 0.1 # gain for simulating SVEA's ESC

    def update(self, steering: float, velocity: float, **kwds):
        """Updates state.
<<<<<<< HEAD

        Updates state using set sampling time, `dt`, and embedded bicycle
        dynamics. Designed to take same inputs as SVEA vehicle's low-level
        interface.
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)
=======
        
        Designed to take same inputs as SVEA vehicle's low-level interface.
>>>>>>> 4b0286b (More work on simulator)

        Args:
            steering: Input steering angle for the car
            velocity: Input velocity for the car
        """
<<<<<<< HEAD
<<<<<<< HEAD

        # With ESC dynamics
        delta = steering * -1
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
=======

        # With ESC dynamics
        delta = steering
        accel = 1/self.TAU * (velocity - self.vel)

        return super().update(delta, accel, **kwds)
>>>>>>> 4b0286b (More work on simulator)

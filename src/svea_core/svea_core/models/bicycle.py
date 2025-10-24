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

        Args:
            steering: Input steering angle for the car
            velocity: Input velocity for the car
        """

        # With ESC dynamics
        delta = steering
        accel = 1/self.TAU * (velocity - self.vel)

        return super().update(delta, accel, **kwds)

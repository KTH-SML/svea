"""
Adapted from Atsushi Sakai's PythonRobotics pure pursuit example
"""
import math


class PurePursuitController:
    """
    Pure Pursuit controller for following a path.

    This controller computes steering and velocity commands based on the current
    state of the vehicle and a predefined trajectory. It uses a look-ahead distance
    to determine the target point on the trajectory and calculates the necessary
    steering angle and velocity to follow that point.

    Attributes:
        dt: Time step for control updates.
        traj_x: X coordinates of the trajectory points.
        traj_y: Y coordinates of the trajectory points.
        target: Current target point on the trajectory.
        target_velocity: Desired velocity to maintain while following the path.
        error_sum: Integral of velocity error for speed control.
        last_index: Last index used in trajectory calculations.
        is_finished: Flag indicating if the path has been completed.
    Args:
        dt: Time step for control updates, default is 0.01 seconds.
    """

    k = 0.1  # look forward gain
    Lfc = 0.6  # look-ahead distance
    K_p = 0.15  # speed control propotional gain
    K_i = 0.01  # speed control integral gain
    
    # Anti-windup parameters
    max_velocity = 1.0  # [m/s] maximum velocity output
    min_velocity = -1.0  # [m/s] minimum velocity output (for reverse)
    integral_limit = 10.0  # maximum integral error accumulation

    L = 0.324  # [m] wheel base of vehicle

    def __init__(self, dt=0.01):
        self.dt = dt

        self.traj_x = []
        self.traj_y = []
        self.target = None
        # initialize with 0 velocity
        self.target_velocity = 0.0
        self.error_sum = 0.0
        self.last_index = 0
        self.is_finished = False

    def compute_control(self, state, target=None):
        steering = self.compute_steering(state, target)
        velocity = self.compute_velocity(state)
        return steering, velocity

    def compute_steering(self, state, target=None):
        if self.is_finished:
            return 0.0
        if target is None:
            self.find_target(state)
        else:
            # allow manual setting of target
            self.target = target

        x, y, yaw, vel = state

        tx, ty = self.target
        alpha = math.atan2(ty - y, tx - x) - yaw
        if vel < 0:  # back
            alpha = math.pi - alpha
        Lf = self.k * vel + self.Lfc
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / Lf, 1.0)
        return delta

    def compute_velocity(self, state):
        if self.is_finished:
            return 0.0

        x, y, yaw, vel = state

        # speed control with anti-windup
        error = self.target_velocity - vel
        
        # Compute proposed velocity before saturation
        P = error * self.K_p
        I = self.error_sum * self.K_i
        correction = P + I
        proposed_velocity = self.target_velocity + correction
        
        # Apply velocity limits (saturation)
        saturated_velocity = max(self.min_velocity, min(self.max_velocity, proposed_velocity))
        
        # Anti-windup: only integrate if not saturated or if error would reduce saturation
        if (proposed_velocity == saturated_velocity or 
            (proposed_velocity > self.max_velocity and error < 0) or 
            (proposed_velocity < self.min_velocity and error > 0)):
            # Update integral term
            self.error_sum += error * self.dt
            # Clamp integral term to prevent excessive accumulation
            self.error_sum = max(-self.integral_limit, min(self.integral_limit, self.error_sum))
        
        return saturated_velocity

    def find_target(self, state):
        ind = self._calc_target_index(state)
        tx = self.traj_x[ind]
        ty = self.traj_y[ind]
        self.target = (tx, ty)

    def _calc_target_index(self, state):

        x, y, yaw, vel = state

        # search nearest point index
        dx = [x - icx for icx in self.traj_x]
        dy = [y - icy for icy in self.traj_y]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        dist = 0.0
        dynamic_lfc =  min(max(0.8, vel * 0.5), 1.7)
        Lf = max(self.k * vel + dynamic_lfc, 1.2)

        # search look ahead target point index
        while Lf > dist and (ind + 1) < len(self.traj_x):
            dx = self.traj_x[ind + 1] - self.traj_x[ind]
            dy = self.traj_y[ind + 1] - self.traj_y[ind]
            dist += math.sqrt(dx ** 2 + dy ** 2)
            ind += 1

        # terminating condition
        if dist < 1.0:
            self.is_finished = True
            pass

        return ind
"""
Adapted from Atsushi Sakai's PythonRobotics pure pursuit example
"""
import math


class PurePursuitController(object):

    k = 0.6  # look forward gain
    Lfc = 0.2  # look-ahead distance
    K_p = 1.0  # speed control propotional gain
    K_i = 0.2  # speed control integral gain
    L = 0.324  # [m] wheel base of vehicle

    def __init__(self, vehicle_name='', dt=0.01):
        self.vehicle_name = vehicle_name
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
        if target is None:
            self.find_target(state)
        else:
            # allow manual setting of target
            self.target = target

        tx, ty = self.target
        alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
        if state.v < 0:  # back
            alpha = math.pi - alpha
        Lf = self.k * state.v + self.Lfc
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / Lf, 1.0)
        return delta

    def compute_velocity(self, state):
        if self.is_finished:
            return 0.0
        else:
            # speed control
            error = self.target_velocity - state.v
            self.error_sum += error * self.dt
            P = error * self.K_p
            I = self.error_sum * self.K_i
            correction = P + I
            return self.target_velocity + correction

    def find_target(self, state):
        ind = self._calc_target_index(state)
        tx = self.traj_x[ind]
        ty = self.traj_y[ind]
        self.target = (tx, ty)

    def _calc_target_index(self, state):
        # search nearest point index
        dx = [state.x - icx for icx in self.traj_x]
        dy = [state.y - icy for icy in self.traj_y]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        dist = 0.0
        Lf = self.k * state.v + self.Lfc

        # search look ahead target point index
        while Lf > dist and (ind + 1) < len(self.traj_x):
            dx = self.traj_x[ind + 1] - self.traj_x[ind]
            dy = self.traj_y[ind + 1] - self.traj_y[ind]
            dist += math.sqrt(dx ** 2 + dy ** 2)
            ind += 1

        # terminating condition
        if dist < 0.1:
            self.is_finished = True

        return ind

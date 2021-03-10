#!/usr/bin/env python

"""
Module for cooperative models with params set for SVEA cars. This model
can be used for maintaining platoon structures with various
setups of intra-platoon communication.

TODO:
    - Finish documentation of the state-space equation
    - Create nice printout for Laplacian
    - Add support for arbitrary communication structures, currently the
      model creation only supports k-nearest neighbor communication.
"""

import numpy as np

__license__ = "MIT"
__maintainer__ = "Frank Jiang"
__email__ = "frankji@kth.se"
__status__ = "Development"


class C_OVRV(object):
    """
    The cooperative version of the optimal-velocity-relative-velocity
    (OVRV) model, aka the C-OVRV model. The model dictates the
    longitudinal speed of a given vehicle in a platoon based on the
    communicated state of the k-nearest platoon members. Specifically,
    the expected communicated values are the forward space-gap, velocity
    and length of each of the communicating vehicles. This C-OVRV model
    has the following state-space equation:

        .. math::

            \\dot{s} &= R\\dot{x} + ce_1, \\\\
            \\ddot{x} &= (K_1+K_4D)s + M\\dot{x} - K_1\\nu - K_4D(\\nu + l) + w.

    where :math:`s, \dot{x}` are the space-gap between a vehicle and the
    vehicle in front of it and the longitudinal speed of the vehicle.
    Units are [m, s, m/s].

    This model is meant to be used at steady-state. In other words, the
    platoon should already be formed and travelling at the intended
    speed.

    :param platoon_size: Number of vehicles in platoon.
    :type platoon_size: int
    :param num_neighbors: Number of forward neighbors communicating
    with initially.
    :type num_neighbors: int
    :param k_gains: Gains, k_1, k_2, k_3, k_4, error will be thrown if
    given number of values is not 4.
    :type k_gains: list
    :param min_spacing: Jam or minimum spacing, corresponds to
    :math: `\eta` in [m].
    :type min_spacing: float
    :param time_headway: Desired time headway, corresponds to
    :math:`\tau` in [s].
    :type time_headway: float
    :param init_leader_vel: Initial velocity of leader vehicle in [m/s].
    :type init_leader_vel: float
    :param length: Bumper-to-bumper length in [m], defaults to 0.586
    (length of SVEA).
    :type length: float
    :param dt: Sampling time for dynamics update in [s], defaults to
    0.01
    :type dt: float
    """

    def __init__(self, platoon_size, num_neighbors, k_gains, min_spacing,
                       time_headway, init_leader_vel, length=0.586, dt=0.01):
        """ Initialize state. """


        self.curr_platoon_size = platoon_size
        self.curr_num_neighbors = num_neighbors

        assert len(k_gains) == 4
        self.k_gains = k_gains
        self.min_spacing = min_spacing
        self.time_headway = time_headway
        self.leader_vel = init_leader_vel
        self.length = length
        self.dt = dt

        self._update_system_dynamics()

    def _build_KNN_laplacian(self, n, k):
        A = np.zeros((n, n))
        for i in range(n):
            for j in range(i):
                if (i-j) < k+1:
                    A[i, j] = 1

        D = np.diag(np.sum(A, axis=1))
        L = D - A

        for i in range(n):
            for j in range(i-1, 0, -1):
                D[i, j-1] = D[i, j] -1
        D[D<0] = 0

        return L, D

    def _update_system_dynamics(self):
        """
        Updates dynamical equation based on given platoon size and
        connectivity situation, also computes equilibrium point given
        these conditions.
        """

        n = self.curr_platoon_size
        k = self.curr_num_neighbors

        S, _ = self._build_KNN_laplacian(n, 1)
        S = -S
        S[0, 0] = -1

        L, D = self._build_KNN_laplacian(n, k)

        I = np.eye(n)
        T = self.time_headway * I

        k1, k2, k3, k4 = self.k_gains
        M = k2*S - k3*L - k1*T - k4*np.dot(D, T)

        # assemble new linear system
        self.A = np.block([
            [np.zeros((n, n)),    S],
            [k1*I + k4*D,       M]
        ])
        self.b = np.append(np.zeros((n, 1)),
            -k1*self.min_spacing*np.ones((n, 1)) -
            np.dot(k4*D*(self.min_spacing+self.length), np.ones((n, 1))))

        # compute current equilibrium point
        eq_pt = np.append(
            np.linalg.lstsq(
                k1*I + k4*D,
                k1*self.min_spacing*np.ones((n, 1))
                + k4*np.dot(D, self.min_spacing*np.ones((n, 1))
                               + self.length*np.ones((n, 1)))
                - self.leader_vel*np.dot(M, np.ones((n, 1))),
                rcond=-1
            )[0],
            self.leader_vel*np.ones((n, 1)),
        )
        self.equilibrium_pt = eq_pt

    def _build_param_printout(self):
        # TODO: write a cool printout expressing current adjacency matrix and
        # laplacian
        pass

    def __repr__(self):
        return self._build_param_printout()
    def __str__(self):
        return self._build_param_printout()

    def compute_accel(self, spaces, velocities, leader_v):
        """
        Given current spacing and velocity in the platoon, compute the
        acceleration based on the C-OVRV model.

        :param spaces: Spaces in front of each vehicle in the platoon,
        must be same length as the size of the platoon.
        :type spaces: list
        :param velocities: Velocity of each vehicle in the platoon,
        must be same length as the size of the platoon.
        :type velocities: list
        :param leader_v: Current velocity of the vehicle in front of the
        platoon.
        :type leader_v: float
        :return: Accelerations for each vehicle to properly maintain the
        platoon. These can be considered control inputs.
        :rtype: list
        """
        assert len(spaces) == self.curr_platoon_size \
               and len(velocities) == self.curr_platoon_size
        assert type(spaces) == type([]) and type(velocities) == type([])

        state = np.array(spaces+velocities)
        self.b[0] = leader_v
        out = np.dot(self.A, state) + self.b

        accels = out[self.curr_platoon_size:]
        return accels

    def update_leader_vel(self, new_vel):
        """
        Update leader velocity [m/s] for updating equilibrium of C-OVRV
        model.

        :param new_vel: New velocity of the vehicle in front of the
        platoon (leader).
        :type new_vel: float
        """
        self.leader_vel = new_vel
        self._update_system_dynamics()

    def update_platoon_size(self, new_size):
        """
        Update platoon size for updating dynamics and equilibrium point
        of C-OVRV model.

        :param new_size: New platoon size.
        :type new_size: int
        """
        self.curr_platoon_size = new_size
        self._update_system_dynamics()

    def update_k_neighbors(self, new_k):
        """
        Update number of communicating neighbors (in the forward
        direction) for updating dynamics and equilibrium point of C-OVRV
        model.

        :param new_k: New number of communicating neighbors.
        :type new_k: int
        """
        self.curr_num_neighbors = new_k
        self._update_system_dynamics()

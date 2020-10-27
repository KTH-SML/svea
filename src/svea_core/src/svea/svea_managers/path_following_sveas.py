#!/usr/bin/env python

"""
Module containing path following SVEA managers
"""

import math

from svea_archetypes import SVEAManager
# from svea.simulators.viz_utils import publish_path, publish_target
from svea.data import TrajDataHandler
from svea.controllers import pure_pursuit

## PURE PURSUIT PARAMS ########################################################
pure_pursuit.k = 0.6  # look forward gain
pure_pursuit.Lfc = 0.4  # look-ahead distance
pure_pursuit.L = 0.324  # [m] wheel base of vehicle
###############################################################################

__license__ = "MIT"
__maintainer__ = "Frank Jiang, Tobias Bolin"
__email__ = "frankji@kth.se "
__status__ = "Development"


class SVEAPurePursuit(SVEAManager):
    """Container for the different software module of a SVEA
    vehicle that is performing path following using a pure-pursuit
    algorithm.

    :param vehicle_name: Name of vehicle; used to initialize each
                         software module.
    :type vehicle_name: str
    :param localizer: A chosen localization interface class constructor
    :type localizer: class
    :param controller: A chosen controller class constructor
    :type controller: class
    :param traj_x: X coordinates of trajectory
    :type traj_x: list
    :param traj_y: Y coordinates of trajectory
    :type traj_y: list
    :param data_handler: A chosen data handler class constructor,
                         defaults to TrajDataHandler
    :type data_handler: class
    """

    def __init__(self, vehicle_name, localizer, controller,
                       traj_x, traj_y, data_handler=TrajDataHandler):

        SVEAManager.__init__(self, vehicle_name, localizer, controller,
                                   data_handler = data_handler)

        self.update_traj(traj_x, traj_y)

        # goto parameters
        self.goto_vel = 0.6 # m/s
        self.goto_thresh = 0.05 # m

    def compute_control(self, state=None):
        """Compute control for path-following using pure-pursuit

        :param state: State used to compute control; if no state is
                      given as an argument, self.state is automatically
                      used instead, defaults to None
        :type state: VehicleState, or None

        :return: Computed steering and velocity inputs from pure-pursuit
                 algorithm
        :rtype: float, float
        """
        if state is None:
            steering, velocity = self.controller.compute_control(self.state)
            self.data_handler.update_target(self.controller.target)
        else:
            steering, velocity = self.controller.compute_control(state)
            self.data_handler.update_target(self.controller.target)
        return steering, velocity

    def goto_pt(self, pt):
        """Compute control to go to single point, taking advantage of
        the pure-pursuit controller

        :param pt: Point to go to
        :type pt: tuple
        :return: Computed steering and velocity inputs from pure-pursuit
                 algorithm
        :rtype: float, float
        """
        curr_xy = [self.state.x, self.state.y]
        target_xy = (pt[0], pt[1])
        dist = math.sqrt((curr_xy[0] - target_xy[0])**2
                         + (curr_xy[1] - target_xy[1])**2)

        if dist > self.goto_thresh:
            steering, velocity = \
                self.controller.compute_control(self.state, target_xy)
            self.data_handler.update_target(self.controller.target)
            return steering, velocity
        else:
            steering = 0.0
            velocity = 0.0
            return steering, velocity

    def update_traj(self, traj_x, traj_y):
        """Update trajectory

        :param traj_x: X coordinates of trajectory, defaults to []
        :type traj_x: list
        :param traj_y: Y coordinates of trajectory, defaults to []
        :type traj_y: list
        """

        assert len(traj_x) == len(traj_y)
        self.controller.traj_x = traj_x
        self.controller.traj_y = traj_y
        self.data_handler.update_traj(traj_x, traj_y)

    @property
    def is_finished(self):
        """Check if pure-pursuit controller is finished or not"""
        return self.controller.is_finished

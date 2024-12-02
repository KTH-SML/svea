#!/usr/bin/env python

"""
Module containing path following SVEA managers
"""

import math

from .svea_archetypes import SVEAManager
# from svea.simulators.viz_utils import publish_path, publish_target
from svea.data import TrajDataHandler
from svea.controllers import pure_pursuit
from svea.controllers.pure_pursuit import PurePursuitController
from svea.interfaces import ActuationInterface
from svea.sensors import Lidar
from svea.data import BasicDataHandler

__license__ = "MIT"
__maintainer__ = "Frank Jiang, Tobias Bolin"
__email__ = "frankji@kth.se "
__status__ = "Development"


class SVEAPurePursuit(SVEAManager):
    """Container for the different software module of a SVEA
    vehicle that is performing path following using a pure-pursuit
    algorithm.

    :param localizer: A chosen localization interface class constructor
    :type localizer: class
    :param controller: A chosen controller class constructor, this
    should be a pure pursuit based controller. This has no default
    controller so the specific pure pursuit controller has to be
    specified
    :type controller: class
    :param traj_x: X coordinates of trajectory
    :type traj_x: list
    :param traj_y: Y coordinates of trajectory
    :type traj_y: list
    :param data_handler: A chosen data handler class constructor,
                         defaults to TrajDataHandler
    :type data_handler: class
    :param vehicle_name: Name of vehicle; used to initialize each
                         software module, defaults to ''
    :type vehicle_name: str
    """

    def __init__(self, localizer, controller, traj_x, traj_y,
                 data_handler=TrajDataHandler, vehicle_name=''):

        SVEAManager.__init__(self, localizer, controller,
                             data_handler = data_handler,
                             vehicle_name = vehicle_name)

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
            self.controller.target_velocity = self.goto_vel
            steering, velocity = \
                self.controller.compute_control(self.state, target_xy)
            self.data_handler.update_target(self.controller.target)
            return steering, velocity
        else:
            self.controller.target_velocity = 0.0
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


class SVEAPlatoonMember(SVEAPurePursuit):
    """Special case of SVEA pure pursuit object that is convenient for
    controlling a platoon vehicle.

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
    :param vehicle_name: Name of vehicle; used to initialize each
                         software module, defaults to ''
    :type vehicle_name: str
    """

    def __init__(self, localizer, traj_x, traj_y, data_handler=TrajDataHandler,
                 vehicle_name=''):

        SVEAPurePursuit.__init__(self, localizer,
                                 PurePursuitController,
                                 traj_x, traj_y,
                                 data_handler = data_handler,
                                 vehicle_name = vehicle_name)

    def send_vel(self, vel):
        self.controller.target_velocity = vel
        steering, velocity = self.compute_control()
        self.send_control(steering, velocity)

    def send_accel(self, accel, dt):
        self.controller.target_velocity += accel * dt
        steering, velocity = self.compute_control()
        self.send_control(steering, velocity)



class SVEAManagerMPC(SVEAManager):
    """Extended SVEAManager class with an additional controller_config_path parameter used to ease MPC setup.
    
    This class inherits from SVEAManager and adds the capability to pass a
    configuration path specifically to the controller.

    The __init__ method is redefined to delay the creation of the controller,
    allowing the controller to be initialized with both vehicle_name and
    controller_config_path, which is not supported in the base SVEAManager.

    :param localizer: A chosen localization interface class constructor
    :type localizer: class
    :param controller: A chosen controller class constructor
    :type controller: class
    :param actuation: A chosen actuation interface class constructor,
                      defaults to ActuationInterface
    :type actuation: class
    :param data_handler: A chosen data handler class constructor,
                         defaults to BasicDataHandler
    :type data_handler: class
    :param vehicle_name: Name of vehicle; used to initialize each
                         software module, defaults to ''
    :type vehicle_name: str
    :param controller_config_path: Path to the configuration file for the controller,
                                   defaults to ''
    :type controller_config_path: str
    """
    
    def __init__(self, localizer, controller, 
                 actuation=ActuationInterface, 
                 data_handler=BasicDataHandler, 
                 vehicle_name='', 
                 controller_config_path=''):
        
        # Manually initialize components from SVEAManager except for the controller
        self.vehicle_name = vehicle_name
        sub_namespace = vehicle_name + '/' if vehicle_name else ''
        self._emergency_topic = '{}lli/emergency'.format(sub_namespace)

        # Initialize localizer, actuation, and data handler
        self.localizer = localizer(vehicle_name)
        self.actuation = actuation(vehicle_name)
        self.data_handler = data_handler(vehicle_name)

        # Initialize the controller with the additional controller_config_path parameter
        self.controller = controller(vehicle_name, controller_config_path)

        # Initialize additional components from SVEAManager
        self.lidar = Lidar()

        # Bring localizer state out into manager
        self.state = self.localizer.state
        self.last_state_time = None

        # Set up automatic state logging
        self.localizer.add_callback(self.data_handler.log_state)
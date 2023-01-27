#!/usr/bin/env python

"""
Module containing archetypal SVEA manager classes.
"""

from copy import deepcopy

import rospy
from svea.interfaces import ActuationInterface
from svea.sensors import Lidar
from svea.data import BasicDataHandler

__license__ = "MIT"
__maintainer__ = "Frank Jiang"
__email__ = "frankji@kth.se "
__status__ = "Development"


class SVEAManager(object):
    """Container for the different software module of a SVEA
    vehicle. Additionally, this manager is used to provide clean
    interfaces to each module of a SVEA vehicle. In general, every SVEA
    consists of the following modules:
    1. Localization Interface
    2. Controller
    3. Actuation Interface
    4. Data Handler
    Note, we use the term "Interface" carefully. They are called
    interfaces since the main functionality is performed elsewhere in
    another ROS node.

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
    """

    MAX_WAIT = 1.0/10.0 # no slower than 10Hz
    SAFETY_HORIZON = 1.0 # look one second ahead to check safety

    def __init__(self, localizer, controller,
                       actuation=ActuationInterface,
                       data_handler=BasicDataHandler,
                       vehicle_name=''):

        self.vehicle_name = vehicle_name
        sub_namespace = vehicle_name + '/' if vehicle_name else ''
        self._emergency_topic = '{}lli/emergency'.format(sub_namespace)

        self.localizer = localizer(vehicle_name)
        self.controller = controller(vehicle_name)
        self.actuation = actuation(vehicle_name)
        self.data_handler = data_handler(vehicle_name)

        self.lidar = Lidar()

        # bring localizer state out into manager
        self.state = self.localizer.state
        self.last_state_time = None

        # set up automatic state logging
        self.localizer.add_callback(self.data_handler.log_state)


    def start(self, wait=False):
        """ Start interface objects

        :param wait: Flag deciding whether to wait for actuation
                     interface to be ready or not
        :type wait: bool
        """
        self.localizer.start()
        self.lidar.start()
        self.actuation.start(wait)

    def wait_for_state(self):
        """Wait for a new state to arrive, or until a maximum time
        has passed since the last state arrived.

        :return: New state when it arrvies, if it arrives before max
                 waiting time, otherwise None
        :rtype: VehicleState, or None
        """
        time = rospy.get_time()
        if self.last_state_time is None:
            timeout = None
        else:
            timeout = self.MAX_WAIT - (time - self.last_state_time)
        if timeout is None or timeout <= 0:
            return deepcopy(self.state)

        self.localizer.ready_event.wait(timeout)
        wait = rospy.get_time() - time
        if wait < self.MAX_WAIT:
            return deepcopy(self.state)
        else:
            return None

    def compute_control(self, state=None):
        """Compute control using chosen controller object

        :param state: State used to compute control; if no state is
                      given as an argument, self.state is automatically
                      used instead, defaults to None
        :type state: VehicleState, or None
        :return: Computed control input; format depends on return type
                 of controller object's own compute_control() method;
                 typically this should return a steering angle and a
                 velocity
        :rtype: Some return type as self.controller.compute_control()
        """
        if state is None:
            return self.controller.compute_control(self.state)
        return self.controller.compute_control(state)

    @property
    def low_level_is_emergency(self):
        """Checks if the firmware is in emergency lock or not"""
        return self.actuation.emergency

    def send_control(self, steering, velocity, transmission=-1):
        """Send control input off to low-level controller and actuate
        SVEA.

        :param steering: Steering angle request in [rad]
        :type steering: float
        :param velocity: Velocity request in [m/s]
        :type velocity: float
        :param transmission: Transmission resquest [0 or 1]
        :type transmission: int
        """
        self.actuation.send_control(steering, velocity, transmission)
        # log control as soon as it's actually sent
        self.data_handler.log_ctrl(steering, velocity, rospy.get_time())

    def _log_state(self):
        self.data_handler.log_state(self.state)

    def visualize_data(self):
        """Visualize data using the visualize_data() method of the
        chosen data handler object"""
        self.data_handler.visualize_data()

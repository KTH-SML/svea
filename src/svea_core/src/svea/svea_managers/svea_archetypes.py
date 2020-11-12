#!/usr/bin/env python

"""
Module containing archetypal SVEA manager classes.
"""

from math import cos, sin, sqrt
from copy import deepcopy
from random import randint
import numpy as np

import rospy
from svea_msgs.msg import lli_emergency
from svea.actuation import ActuationInterface
from svea.sensors import Lidar
from svea.states import VehicleState
from svea.data import BasicDataHandler
from svea.models.bicycle import SimpleBicycleModel
from svea.reachable_sets.reach_util import (load_TTR_from_mat,
                                            check_safe_with_control)

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

    :param vehicle_name: Name of vehicle; used to initialize each
                         software module.
    :type vehicle_name: str
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
    """

    MAX_WAIT = 1.0/10.0 # no slower than 10Hz
    SAFETY_HORIZON = 1.0 # look one second ahead to check safety

    def __init__(self, vehicle_name, localizer, controller,
                       actuation=ActuationInterface,
                       data_handler=BasicDataHandler):

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

        # load emergency brake reachable set
        self._emergency_checker_init = False
        self._init_emergency_check()
        self.calling_emergency = True # more convenient at startup

    def _init_emergency_check(self):
        self._TTR_set = load_TTR_from_mat('TTR_and_grad.mat')
        self._nearest_obstacle_TTR = float('inf') # not close to an obstacle
        self._emergency_pub = rospy.Publisher(self._emergency_topic,
                                              lli_emergency,
                                              queue_size=1,
                                              tcp_nodelay=True)
        self._emergency_id = randint(0, 1000)
        self._emergency_checker_init = True

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
        if timeout <= 0:
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

    def future_rel_states(self, steering, velocity):
        """Based on a bicycle model, computes the future relative
        states of the vehicle within an adaptive time horizon (based
        on the vehicle's velocity), given a fixed control. Primarily
        used for checking safety.

        :param steering: Requested steering
        :type steering: float
        :param velocity: Requested velocity
        :type velocity: float
        :param horizon: Horizon to simulate over
        :type horizon: float
        :return: Future relative states assuming fixed steering and
        velocity
        :rtype: list
        """
        dt = 0.075
        horizon = self.SAFETY_HORIZON / (abs(self.state.v) + 0.5)
        init_state = VehicleState()
        init_state.array = [0.0, 0.0, 0.0, self.state.v]
        model = SimpleBicycleModel(init_state)
        next_states = []
        for _ in range(int(horizon/dt)):
            model.update(steering, velocity, dt)
            next_state = deepcopy(model.state)
            next_states.append(next_state.array)
        return next_states

    def _safety_check(self, steering, velocity):
        """Checks safety of vehicle based on a backward reachable set
        starting from each lidar scan point (if they are within 2
        meters, otherwise vehicle is safe). The reachable set is
        computed offline with the Level Set Toolbox and is the solution
        to an avoid formulation of the Hamilton-Jacobi-Issaacs
        inequality.

        In addition to checking whether the vehicle is currently safe
        and will be safe under the current control input, this method
        also reports the "primary" reason why the vehicle is unsafe.
        When there is a safety problem find in the lidar scans, this is
        done by seeing why the majority of unsafe lidar points are
        unsafe.

        :param steering: Steering to check future safety
        :type steering: float
        :param velocity: Velocity to check future safety
        :type velocity: float
        :return: Whether requested steering and velocity are safe or not
        and the primary reason why vehicle is unsafe
        :rtype: bool, str
        """
        #TODO: report whether steering or velocity (or both) are the
        #      safety breaching factor

        def should_check(dist):
            return (not dist == float('nan')
                    or not dist == float('inf')
                    or dist < 1)
        def distance(state1, state2):
            return sqrt((state1.x - state2.x)**2 + (state1.y - state2.y)**2)

        reason = "Scans from Lidar not available (currently blind)"
        if len(self.lidar.scan) != 0 and self._emergency_checker_init:
            # compute future states
            future_states = self.future_rel_states(steering, velocity)

            # iterate through lidar scan points to safety check them
            angle_min = self.lidar.angle_min
            angle_increment = self.lidar.angle_increment
            safe_check_results = []
            last_state = VehicleState()
            for i, dist in enumerate(self.lidar.scan):
                # only check reasonably distanced lidar points
                if should_check(dist):
                    angle = angle_min + i*angle_increment
                    rel_state = VehicleState()
                    # relative, thus heading always 0
                    rel_state.array = [-dist * cos(angle), -dist * sin(angle),
                                       0.0, self.state.v]
                    # only check points further than 5cm away from each other
                    if distance(rel_state, last_state) > 0.05:
                        result, curr_TTR = check_safe_with_control(
                            self._TTR_set, rel_state, future_states)
                        safe_check_results.append(result)
                        last_state = rel_state
                        if result == "Unsafe" or result == "Ctrl Unsafe":
                            break
            if len(safe_check_results) != 0:
                results = np.array(safe_check_results)
                is_safe = np.logical_or(results == "Safe", results == "Ctrl Safer")
                # check if all lidar points are safe
                if not np.all(is_safe):
                    num_ctrl_unsafe = np.sum(results == "Ctrl Unsafe")
                    num_state_unsafe = np.sum(results == "Unsafe")
                    reason = "Requested control leads to unsafe state" \
                            if num_ctrl_unsafe > num_state_unsafe \
                            else "Point(s) detected by lidar are unsafe states"
                else:
                    reason = "Vehicle is safe"
                return np.all(is_safe), reason
            else:
                reason = "No points in range of lidar"
                return True, reason
        return False, reason

    @property
    def low_level_is_emergency(self):
        """Checks if the firmware is in emergency lock or not"""
        return self.actuation.emergency

    def send_control(self, steering, velocity):
        """Send control input off to low-level controller and actuate
        SVEA. If the control signal is found to be unsafe, it will be
        rejected and an emergency flag will be triggered. This method
        also auto-logs the control input and when emergencies start and
        end.

        :param steering: Steering angle request in [rad]
        :type steering: float
        :param velocity: Velocity request in [m/s]
        :type velocity: float
        """
        # check if requested control is safe
        # is_safe, reason = self._safety_check(steering, velocity)
        is_safe = True
        reason = "Emergency brake currently disabled"
        if is_safe:
            if self.calling_emergency: # if emergency WAS called
                is_start = False # log the end of emergency
                self.data_handler.log_emergency(is_start, reason,
                                                rospy.get_time())
            emergency_msg = lli_emergency(False, self._emergency_id)
            self._emergency_pub.publish(emergency_msg)
            self.calling_emergency = False

            self.actuation.send_control(steering, velocity)
            # log control as soon as it's actually sent
            self.data_handler.log_ctrl(steering, velocity, rospy.get_time())
        else:
            if len(self.lidar.scan) != 0 and self._emergency_checker_init:
                if not self.calling_emergency:
                    emergency_msg = lli_emergency(True, self._emergency_id)
                    self._emergency_pub.publish(emergency_msg)
                    self.calling_emergency = True
                    rospy.loginfo("Rejecting send_control: {}".format(reason))
                    # log the start of the emergency
                    is_start = True
                    self.data_handler.log_emergency(is_start, reason,
                                                    rospy.get_time())

    def _log_state(self):
        self.data_handler.log_state(self.state)

    def visualize_data(self):
        """Visualize data using the visualize_data() method of the
        chosen data handler object"""
        self.data_handler.visualize_data()

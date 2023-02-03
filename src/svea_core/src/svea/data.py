#!/usr/bin/env python

"""
Module containing all the handy data handlers for simplifying and
standardizing all things data. Data handlers handle data logging, data
accessing, data visualizing, and saving data.
"""

import math
import pickle
from collections import deque
import rospy
from copy import deepcopy
import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.style.use('seaborn-muted')

from geometry_msgs.msg import PolygonStamped, PoseStamped, PointStamped
from nav_msgs.msg import Path

from svea.states import VehicleState
from svea.simulators.viz_utils import (
    publish_path,
    publish_3Dcar,
    publish_target,
    plot_car,
    lists_to_pose_stampeds)

__license__ = "MIT"
__maintainer__ = "Frank Jiang"
__email__ = "frankji@kth.se "
__status__ = "Development"


class BasicDataHandler(object):
    """ Handler for the basic data-related tasks, including logging, plotting,
    and saving data. Also a good class to inherit from to standardize the names
    of common tasks such as `visualize_data`.

    :param vehicle_name: Name of vehicle being controlled; unused in this
                         class, but included as a standard, defaults to ''
    :type vehicle_name: str, optional
    """

    def __init__(self, vehicle_name=''):
        self.vehicle_name = vehicle_name
        self.start_time = rospy.get_time()

        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

        # start with zeros since SVEA does this at launch
        self.ctrl_steer = [0.0]
        self.ctrl_v = [0.0]
        self.ctrl_trans = [-1]
        self.ctrl_t = [0.0]

        # emergencies record
        self.emergency_start_end = [] # 1 start, 0 end
        self.emergency_reasons = []
        self.emergency_t = []

        self._last_published_state = None

    @property
    def curr_state(self):
        """ Most recent state of the vehicle

        :return: Last logged state of vehicle
        :rtype: VehicleState
        """
        state = VehicleState(x = self.x[-1],
                             y = self.y[-1],
                             yaw = self.yaw[-1],
                             v = self.v[-1])
        return state

    @property
    def curr_steer(self):
        """ Most recent steering input of the vehicle

        :return: Last logged steering of vehicle
        :rtype: float
        """
        # important for plotting
        return self.ctrl_steer[-1]

    def log_state(self, state):
        """ Log state

        :param state: New state to log
        :type vehicle_name: VehicleState
        """
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(state.time_stamp.to_sec()-self.start_time)

    def log_ctrl(self, steering, v, t, trans=-1):
        """ Log control input and time of control at time of request

        :param steering: Steering input
        :type steering: float
        :param v: Velocity input
        :type v: float
        :param t: Timestamp of control input [s]
        :type t: float
        :param trans: Transmission input
        :type trans: int
        """
        self.ctrl_steer.append(steering)
        self.ctrl_v.append(v)
        self.ctrl_trans.append(trans)
        self.ctrl_t.append(t-self.start_time)

    def log_emergency(self, is_start, reason, t):
        """ Log when emergencies are called and ended

        :param is_start: Whether it's start of emergency or not
        :type is_start: bool
        :param t: Timestame of emergency [s]
        :type t: float
        """
        self.emergency_start_end.append(int(is_start))
        self.emergency_reasons.append(reason)
        self.emergency_t.append(t)

    def plot_data(self):
        """ Plot data collected so far """

        curr_state = self.curr_state
        curr_steer = self.curr_steer
        log_x = self.x
        log_y = self.y

        plot_car(curr_state.x,
                 curr_state.y,
                 curr_state.yaw,
                 curr_steer)

        plt.plot(log_x, log_y, ".r", label="course")

        plt.axis("equal")
        plt.grid(True)
        plt.title("Heading[deg]: "
                + str(math.degrees(curr_state.yaw))[:4]
                + " | Speed[m/s]:"
                + str(curr_state.v)[:4])

    def update_animation(self, only_plot=True):
        """ Clear and update animation between each frame

        :param only_plot: Flag used when several processes want to plot
                          on same figure. If set to True, other processes will
                          not be able to plot. If set to False, other processes
                          can plot in figure without being cleared out,
                          defaults to True.
        :type only_plot: bool
        """
        if only_plot:
            plt.cla() # clear last plot/frame

        self.plot_data()

        if only_plot:
            plt.pause(0.001)

    def visualize_data(self, only_plot=True):
        """ Visualize logged data

        :param only_plot: Flag used when several processes want to plot
                          on same figure. If set to True, other processes will
                          not be able to plot. If set to False, other processes
                          can plot in figure without being cleared out,
                          defaults to True
        :type only_plot: bool
        """

        if len(self.t) != 0 and len(self.ctrl_t) != 0:
            self.update_animation(only_plot)

    def save_data(self, filename):
        """ Save logged data

        :param filename: Path to desired save location
        :type filename: str
        """
        data_dict = self.__dict__
        file_handler = open(filename, 'w')
        pickle.dump(data_dict, file_handler)


class TrajDataHandler(BasicDataHandler):
    """ Handler with additional features for handling trajectories. Role of
    class is primarily for handling trajectory plotting.

    :param vehicle_name: Name of vehicle being controlled; unused in this
                         class, but included as a standard, defaults to ''
    :type vehicle_name: str, optional
    :param traj_x: X coordinates of trajectory, defaults to []
    :type traj_x: list
    :param traj_y: Y coordinates of trajectory, defaults to []
    :type traj_y: list
    """

    def __init__(self, vehicle_name='', traj_x=[], traj_y=[]):
        BasicDataHandler.__init__(self, vehicle_name)

        self.traj_x = traj_x
        self.traj_y = traj_y
        self.target = None

    def update_traj(self, traj_x, traj_y):
        """ Update trajectory

        :param traj_x: X coordinates of trajectory, defaults to []
        :type traj_x: list
        :param traj_y: Y coordinates of trajectory, defaults to []
        :type traj_y: list
        """
        self.traj_x = traj_x
        self.traj_y = traj_y

    def update_target(self, target):
        """Update target

        :param target: tuple containing XY coordinate of vehicle's target
        :type target: tuple
        """
        self.target = target

    def plot_data(self):
        """ Plot data collected so far, with addition of trajectories.
        """
        BasicDataHandler.plot_data(self)

        plt.plot(self.traj_x, self.traj_y, "-b", label="trajectory")
        if not self.target is None:
            plt.plot(self.target[0],
                     self.target[1],
                     "xg", label="target")

        plt.axis("equal")

    def save_data(self, filename):
        """ Save logged data

        :param filename: Path to desired save location
        :type filename: str
        """
        # currently does not handle multiple trajectories
        data_dict = self.__dict__
        data_dict["traj_x"] = self.traj_x
        data_dict["traj_y"] = self.traj_y
        file_handler = open(filename, 'w')
        pickle.dump(data_dict, file_handler)

class RVIZPathHandler(TrajDataHandler):
    """ Handler with additional features for handling visualization to
    RVIZ.

    :param vehicle_name: Name of vehicle being controlled; unused in this
                         class, but included as a standard, defaults to ''
    :type vehicle_name: str, optional
    :param traj_x: X coordinates of trajectory, defaults to []
    :type traj_x: list
    :param traj_y: Y coordinates of trajectory, defaults to []
    :type traj_y: list
    """

    TRAVEL_DIST_THRESH = 0.1
    PATH_MAX_LEN = 1000

    def __init__(self, vehicle_name='', traj_x=[], traj_y=[]):
        TrajDataHandler.__init__(self, vehicle_name, traj_x, traj_y)
        self.past_path = deque(maxlen=self.PATH_MAX_LEN)
        self.sub_namespace = vehicle_name + '/' if vehicle_name else ''
        poly_topic = self.sub_namespace + "3D_car"
        self.car_poly_pub = \
            rospy.Publisher(poly_topic, PolygonStamped, queue_size=1)
        path_topic = self.sub_namespace + "path_plan"
        self.path_plan_pub = \
            rospy.Publisher(path_topic, Path, queue_size=1, latch=True)
        past_topic = self.sub_namespace + "past_path"
        self.past_path_pub = \
            rospy.Publisher(past_topic, Path, queue_size=1, latch=True)
        pose_topic = self.sub_namespace + "vis_pose"
        self.pose_pub = \
            rospy.Publisher(pose_topic, PoseStamped, queue_size=1)
        target_topic = self.sub_namespace + "target"
        self.target_pub = \
            rospy.Publisher(target_topic, PointStamped, queue_size=1, latch=True)

    @property
    def _should_update_path(self):
        curr_state = self.curr_state

        if self._last_published_state is None:
            self._last_published_state = deepcopy(curr_state)
            return True

        prev_x = self._last_published_state.x
        prev_y = self._last_published_state.y
        pose_diff = ((curr_state.x - prev_x)**2
                     + (curr_state.y - prev_y)**2)**0.5

        if pose_diff > self.TRAVEL_DIST_THRESH:
            self._last_published_state = deepcopy(curr_state)
            return True
        else:
            return False

    def update_traj(self, traj_x, traj_y):
        """ Update trajectory, but here updated path is also published
        to RVIZ.

        :param traj_x: X coordinates of trajectory, defaults to []
        :type traj_x: list
        :param traj_y: Y coordinates of trajectory, defaults to []
        :type traj_y: list
        """
        TrajDataHandler.update_traj(self, traj_x, traj_y)
        publish_path(self.path_plan_pub, self.traj_x, self.traj_y)

    def pub_path(self):
        """ Publish path """
        new_path = lists_to_pose_stampeds(self.x[-1:], self.y[-1:])
        self.past_path += new_path
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'map'
        path.poses = self.past_path
        self.past_path_pub.publish(path)

    def pub_target(self):
        """ Publish target """
        if not self.target is None:
            target_x = self.target[0]
            target_y = self.target[1]
            publish_target(self.target_pub, target_x, target_y)

    def pub_car_poly(self):
        """ Publish car polygon """
        publish_3Dcar(self.car_poly_pub,
                      self.pose_pub,
                      self.x[-1],
                      self.y[-1],
                      self.yaw[-1])

    def visualize_data(self):
        """ Visualize data only after data has been logged and only
        publish path when the distance traveled criteria has been met
        """
        if len(self.t) != 0 and len(self.ctrl_t) != 0:
            self.pub_car_poly()
            self.pub_target()
            if self._should_update_path:
                self.pub_path()

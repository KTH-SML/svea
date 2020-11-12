#!/usr/bin/env python

"""
LiDAR module that contains ROS interface objects for the LiDARs used by
SML. Currently, supporting: RPLidar, (coming soon) Hokuyo
"""

from threading import Thread

import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

__license__ = "MIT"
__maintainer__ = "Frank Jiang"
__email__ = "frankji@kth.se"
__status__ = "Development"


class Lidar():
    """
    Basic interface for handling a Lidar. Collects and stores the most recent
    scan.
    """

    def __init__(self):
        self.scan = []
        # list of functions to call whenever a new scan comes in
        self.callbacks = []

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data

        :return: itself
        :rtype: RPLidar
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Lidar Interface Node: \n" + str(self))
        self._collect_srvs()
        self._start_listen()

    def _collect_srvs(self):
        pass

    def _start_listen(self):
        rospy.Subscriber('scan', LaserScan, self._read_scan)
        rospy.loginfo("Lidar Interface successfully initialized")
        rospy.spin()

    def _read_scan(self, scan_msg):
        self.scan = scan_msg.ranges

        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment

        self.time_increment = scan_msg.time_increment

        self.last_scan_time = scan_msg.scan_time

        for cb in self.callbacks:
            cb(self.scan, self.angle_min, self.angle_increment)

    def add_callback(self, cb):
        """Add state callback. Every function passed into this method
        will be called whenever new scan information comes in from the
        Lidar driver.

        :param cb: A callback function intended for responding to the
                   reception of a new scan, function must accept list
                   of scans, min angle, and angle increment as arguments
        :type cb: function
        """
        self.callbacks.append(cb)

    def remove_callback(self, cb):
        """Remove callback so it will no longer be called when state
        information is received

        :param cb: A callback function that should be no longer used
                   in response to the reception of state info
        :type cb: function
        """
        while cb in self.callbacks:
            self.callbacks.pop(self.callbacks.index(cb))

class RPLidar():
    """
    Interface handling the RPLidar. Collects and stores the most recent
    scan and handles stopping and starting the RPLidar. Implements a
    couple extra feature(s): emergency detection.

    :param emergency_dist: Distance threshold for simple detection of
                           scenarios that could be considered an
                           emergency, defaults to 0.2 [m]
    :type emergency_dist: float
    """

    # options: serial_port, serial_baudrate, frame_id, inverted,
    #          angle_compensate scan_mode
    ROS_PARAM_PREFIX = "rplidarNode"
    IMPORTANT_ROS_PARAMS = ["serial_port", "frame_id",
                            "angle_compensate"]

    def __init__(self, emergency_dist=0.2):
        # rospy.init_node('rplidar_handler')
        self.node_name = "RPLidar A2/A3 Handler"

        self.scan = []

        self.emergency_dist = emergency_dist
        self.is_emergency = False

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data

        :return: itself
        :rtype: RPLidar
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Lidar Interface Node: \n" + str(self))
        self._collect_srvs()
        self._start_listen()

    def _collect_srvs(self):
        rospy.wait_for_service('start_motor')
        self.start_motor_srv = rospy.ServiceProxy('start_motor', Empty)

        rospy.wait_for_service('stop_motor')
        self.stop_motor_srv = rospy.ServiceProxy('stop_motor', Empty)

    def _start_listen(self):
        rospy.Subscriber('scan', LaserScan, self._read_scan)
        rospy.loginfo("Lidar Interface successfully initialized")
        rospy.spin()

    def _read_scan(self, scan_msg):
        self.scan = scan_msg.ranges

        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment

        self.time_increment = scan_msg.time_increment

        self.last_scan_time = scan_msg.scan_time

        self.is_emergency = min(self.scan) < self.emergency_dist

    def _build_param_printout(self):
        param_str = "{0}:\n".format(self.node_name)

        for param_name in self.IMPORTANT_ROS_PARAMS:
            curr_param = rospy.get_param(self.ROS_PARAM_PREFIX+ '/' + param_name)
            param_str += "  - {0}: {1}\n".format(param_name, curr_param)

        return param_str

    def __repr__(self):
        return self._build_param_printout()
    def __str__(self):
        return self._build_param_printout()

    def start_motor(self):
        """
        Physically starts spinning the RPLidar
        """
        try:
            self.start_motor_srv()
        except rospy.ServiceException as exc:
            print(self.node_name + ": Start motor service failed: " + str(exc))

    def stop_motor(self):
        """
        Physically stops the RPLidar from spinning
        """
        try:
            self.stop_motor_srv()
        except rospy.ServiceException as exc:
            print(self.node_name + ": Stop motor service failed: " + str(exc))

    def get_raw_scan(self):
        """
        (will be updated to use properties)

        :return: Most recent lidar scan as list of ranges
        :rtype: list
        """
        return self.scan

    def get_raw_scan_with_time(self):
        """
        (will be updated to use properties)

        :return: Most recent lidar scan as list of ranges along with
                 last scan delay
        :rtype: float, list
        """
        return self.last_scan_time, self.scan

    def get_is_emergency(self):
        """
        (will be updated to use properties)

        :return: Flag indicating whether something is too close or not
        :rtype: bool
        """
        return self.is_emergency

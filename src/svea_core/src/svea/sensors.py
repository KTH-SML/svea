#!/usr/bin/env python

"""
LiDAR module that contains ROS interface objects for the LiDARs used by
SML.
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

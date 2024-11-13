"""
Simulation module for the Lidars. Creates fake ROS publications that
match the publications made by a Lidar driver.
"""

import rospy
import numpy as np
from copy import deepcopy
from math import cos, sin, sqrt, radians
from threading import Thread
from multiprocessing import Pool

from sensor_msgs.msg import LaserScan, PointCloud
from visualization_msgs.msg import Marker

from .viz_utils import publish_lidar_points, publish_lidar_rays, publish_edges

__license__ = "MIT"
__maintainer__ = "Javier Cerna, Frank Jiang"
__email__ = "frankji@kth.se"
__status__ = "Development"


class SimLidar(object):
    """Simulated 1-band lidar. It works by taking a list of obstacles,
    and simulates the detected points. Parameters are based on the
    Hokuyo UST-10LX (reduce computation).

    The position of the vehicle is expected to be set externally
    instead of being updated through a ROS communication channel, this
    is to ensure it can be directly used within the SimSVEA class.

    #TODO: add updating list of visible edges

    :param vehicle_name: Name of vehicle lidar is attached to
    :type vehicle_name: str, optional
    """

    ANGLE_MIN = radians(-135.0) # start angle of scan [rad]
    ANGLE_MAX = radians(135.0) # end angle of the scan [rad]
    # INCREMENT = radians(0.25) # angular distance between measurements [rad]
    INCREMENT = radians(2) # angular distance between measurements [rad]
    TIME_INCREMENT = 0.00002 # time between each beam measurement [s]
    SCAN_TIME = 0.025 # time between scans/between publication [seconds]
    RANGE_MIN = 0.02 # min range of lidar [m]
    # RANGE_MAX = 30.0 # max range of lidar [m]
    RANGE_MAX = 15.0 # max range of lidar [m]

    LIDAR_OFFSET = 0.30 # dist between SVEA rear axle and lidar mount point [m]

    def __init__(self, vehicle_name=''):

        sub_namespace = vehicle_name + '/' if vehicle_name else ''
        self._viz_points_topic = sub_namespace + 'viz_lidar_points'
        self._viz_rays_topic = sub_namespace + 'viz_lidar_rays'
        self._viz_edges_topic = sub_namespace + 'viz_edges'

        if vehicle_name:
            self.vehicle_name = vehicle_name
        else:
            namespace = rospy.get_namespace()
            self.vehicle_name = namespace.split('/')[-2]

        self._lidar_position = None # [x, y, yaw] -> [m, m, rad]
        self._last_visibility_pos = None # [x, y, yaw] -> [m, m, rad]
        self._obstacles = None
        self._visible_edges = []

        self.ranges = []
        self.viz_points = []

        self._pool = Pool(4) # worker pool for tasks that need a bit of help

        self._scan_msg = LaserScan()
        self._scan_msg.header.stamp = rospy.Time.now()
        self._scan_msg.header.frame_id = 'laser'
        self._scan_msg.angle_min = self.ANGLE_MIN
        self._scan_msg.angle_max = self.ANGLE_MAX
        self._scan_msg.angle_increment = self.INCREMENT
        self._scan_msg.time_increment = self.TIME_INCREMENT
        self._scan_msg.scan_time = self.SCAN_TIME
        self._scan_msg.range_min = self.RANGE_MIN
        self._scan_msg.range_max = self.RANGE_MAX

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data

        :return: itself
        :rtype: SimLidar
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Lidar Simulation Node: \n"
                      + str(self))
        self._start_publish()
        rospy.loginfo("{} Simulated Lidar successfully initialized".format(
            self.vehicle_name))
        self._start_simulation()
        rospy.spin()

    def _start_publish(self):
        self._scan_pub = rospy.Publisher('/scan', LaserScan,
                                         queue_size=1, tcp_nodelay=True)
        self._viz_points_pub = rospy.Publisher(self._viz_points_topic,
                                               PointCloud,
                                               queue_size=1)
        self._viz_rays_pub = rospy.Publisher(self._viz_rays_topic,
                                             Marker,
                                             queue_size=1)
        self._viz_edges_pub = rospy.Publisher(self._viz_edges_topic,
                                              Marker,
                                              queue_size=1)

    def _start_simulation(self):
        rate = rospy.Rate(1/self.SCAN_TIME)
        while not rospy.is_shutdown():
            if self._lidar_position is None or self.obstacles is None:
                continue
            self._update_visible_edges()
            self._update_scan()
            self.publish_scan()
            self.publish_viz_points()
            self.publish_viz_rays()
            rate.sleep()

    def update_lidar_position(self, vehicle_state):
        """Updates the lidar position using the vehicle state and the
        known offset between the SVEA rear axle and lidar mount

        :param vehicle_state: State of vehicle lidar is attached to
        :type vehicle_state: VehicleState
        """
        vehicle_xy = np.array([vehicle_state.x, vehicle_state.y])
        yaw = vehicle_state.yaw

        offset = [self.LIDAR_OFFSET, 0.0]
        rot = np.matrix([[np.cos(-yaw), np.sin(-yaw)],
                         [-np.sin(-yaw), np.cos(-yaw)]])
        rot_offset = np.dot(rot, offset).tolist()[0]
        lidar_xy = vehicle_xy + np.array(rot_offset)

        self._lidar_position = np.append(lidar_xy, yaw)

    @property
    def obstacles(self):
        """Grabs simulated obstacles simulated Lidar is using"""
        if self._obstacles is not None:
            return self._obstacles

        obstacles_param = rospy.search_param('obstacles')
        obstacles_from_param = rospy.get_param(obstacles_param, None)

        if type(obstacles_from_param) == type([]):
            self._obstacles = obstacles_from_param
            for obstacle_edges in self._obstacles:
                # add last segment to close polygon
                obstacle_edges += [obstacle_edges[0]]
        else:
            self._obstacles = []
        return self._obstacles

    def _update_visible_edges(self):
        if self._last_visibility_pos is None:
            self._last_visibility_pos = self._lidar_position
        elif np.all(self._last_visibility_pos == self._lidar_position):
            # only update visibilty if lidar has moved
            return

        self._visible_edges = []
        for obstacle_edges in self._obstacles:
            for ind in range(len(obstacle_edges) - 1):
                edge = [obstacle_edges[ind], obstacle_edges[ind + 1]]
                self._visible_edges.append(edge)

        self._last_visibility_pos = deepcopy(self._lidar_position)

    def _update_scan(self):
        """Performs a lidar scan, by computing the closest intersection between
        the obstacles and the beam generated from the lidar (for each angle)."""

        def beam_segment(angle):
            lidar_x = self._lidar_position[0]
            lidar_y = self._lidar_position[1]
            lidar_heading = self._lidar_position[2]
            seg_start = [lidar_x + cos(lidar_heading+angle) * self.RANGE_MIN,
                         lidar_y + sin(lidar_heading+angle) * self.RANGE_MIN]
            seg_end = [lidar_x + cos(lidar_heading+angle) * self.RANGE_MAX,
                       lidar_y + sin(lidar_heading+angle) * self.RANGE_MAX]
            return [seg_start, seg_end]

        angles = np.arange(self.ANGLE_MIN, self.ANGLE_MAX, self.INCREMENT)
        edges = deepcopy(self._visible_edges)
        beams_and_edges = [[beam_segment(angle), edges] for angle in angles]

        dist_and_intersections = self._pool.map(beam_intersection, beams_and_edges)

        self.ranges = []
        self.viz_points = []
        for dist_and_intersection in dist_and_intersections:
            dist = dist_and_intersection[0]
            intersection = dist_and_intersection[1]
            self.ranges.append(dist)
            if not intersection is None:
                self.viz_points.append(intersection)
        self._scan_msg.ranges = self.ranges

    def publish_scan(self):
        self._scan_pub.publish(self._scan_msg)

    def publish_viz_points(self):
        publish_lidar_points(self._viz_points_pub, self.viz_points)

    def publish_viz_rays(self):
        publish_lidar_rays(self._viz_rays_pub, self._lidar_position, self.viz_points)


def beam_intersection(beam_and_edges):
    beam = beam_and_edges[0]
    visible_edges = beam_and_edges[1]
    distance, intersection = _compute_closest_intersection(beam, visible_edges)
    if distance == float('inf'):
        distance = float('nan')
    return [distance, intersection]

def _compute_closest_intersection(beam, visible_edges):
    """Computes all the intersections between a list of obstacles and
    the segment defined by a position and a beam from the lidar. Then,
    it returns the closest one.

    :param position: position of the lidar (x, y)
    :type position: tuple
    :param beam: beam from the lidar (in radians)
    :type angle: float
    :param obstacles: List of obstacles (each one with obstacle edges)
    :type obstacles: list
    :return: closest intersection point (p_x, p_y), if it exists
    :rtype: tuple or None
    """
    closest_distance = float('inf')
    closest_intersection = None
    for edge in visible_edges:
        intersection = _compute_segseg_intersection(beam, edge)
        if not intersection is None:
            distance = _dist(beam[0], intersection)
            if distance < closest_distance:
                closest_distance = distance
                closest_intersection = intersection
    return closest_distance, closest_intersection

def _dist(pt1, pt2):
    return sqrt((pt1[0] - pt2[0]) ** 2
                + (pt1[1] - pt2[1]) ** 2)

def _compute_segseg_intersection(segment1, segment2):
    """Algorithm to compute a segment to segment intersection.
    Based on this article:
        https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection

    :param segment1: first segment (defined by two endpoints)
    :type segment1: list
    :param segment2: second segment (defined by two endpoints)
    :type segment2: list
    :return: intersection point (p_x, p_y), if it exists
    :rtype: tuple or None
    """
    (x1, y1), (x2, y2) = segment1
    (x3, y3), (x4, y4) = segment2

    # Check for parallel lines
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    if denominator == 0:
        return None

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator

    # Check for intersections outside of the segments
    if (t < 0 or t > 1) or (u < 0 or u > 1):
        return None

    p_x = x1 + t * (x2 - x1)
    p_y = y1 + t * (y2 - y1)
    return (p_x, p_y)

def _compute_lineline_intersection(line1_pt1, line1_pt2,
                                    line2_pt1, line2_pt2):
    """Algorithm to compute a line to line intersection, where the
    two lines are both defined by two points. Based on this article:
        https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection

    :param line1_pt1: First point on first line
    :type line1_pt1: tuple
    :param line1_pt2: First point on first line
    :type line1_pt2: tuple
    :param line2_pt1: First point on first line
    :type line2_pt1: tuple
    :param line2_pt2: First point on first line
    :type line2_pt2: tuple
    :return: intersection point (p_x, p_y), if it exists
    :rtype: tuple or None
    """
    (x1, y1) = line1_pt1
    (x2, y2) = line1_pt2
    (x3, y3) = line2_pt1
    (x4, y4) = line2_pt2

    # Check for parallel lines
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    if denominator == 0:
        return None

    p_x = ((x1*y2 - y1*x2) * (x3 - x4) - (x1 - x2) * (x3*y4 - y3*x4)) \
            / denominator
    p_y = ((x1*y2 - y1*x2) * (y3 - y4) - (y1 - y2) * (x3*y4 - y3*x4)) \
            / denominator
    return (p_x, p_y)

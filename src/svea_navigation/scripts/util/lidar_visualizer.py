#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Point32
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from math import sin, cos
from tf.transformations import euler_from_quaternion

from svea.simulators.viz_utils import publish_lidar_points, publish_lidar_rays


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


class VehicleState:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

class LidarVisualizer:
    """Simulated 1-band lidar rays displayer. It works by taking subscribing to the /scan topic.
        It works in simulation as well as on the real car.
    """
    LIDAR_OFFSET_X = 0.284  # X offset of the LIDAR from the base_link
    LIDAR_OFFSET_Y = 0.0    # Y offset of the LIDAR from the base_link
    LIDAR_OFFSET_Z = 0.2    # Z offset of the LIDAR from the base_link (height)

    def __init__(self):
        self._lidar_position = np.array([0.0, 0.0, 0.0])  # x, y, yaw

        rospy.init_node('lidar_visualizer')

        self.IS_SIM = load_param('~is_sim', False)

        if self.IS_SIM:
            state_topic = 'odometry/corrected'               # simulation odometry
        else:
            state_topic = 'odometry/filtered/global'         # real world odometry

        self.rays_publisher = rospy.Publisher('/lidar_rays', Marker, queue_size=10)
        self.point_cloud_publisher = rospy.Publisher('/lidar_points', PointCloud, queue_size=10)

        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber(state_topic, Odometry, self.odometry_callback)

        self.vehicle_state = VehicleState()

    def odometry_callback(self, msg):
        self.vehicle_state.x = msg.pose.pose.position.x
        self.vehicle_state.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.vehicle_state.yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        self.update_lidar_position(self.vehicle_state)


    def update_lidar_position(self, vehicle_state):
        """Updates the lidar position using the vehicle state and the
        known offset between the SVEA rear axle and lidar mount

        :param vehicle_state: State of vehicle lidar is attached to
        :type vehicle_state: VehicleState
        """
        vehicle_xy = np.array([vehicle_state.x, vehicle_state.y])
        yaw = vehicle_state.yaw

        offset = [self.LIDAR_OFFSET_X, self.LIDAR_OFFSET_Y]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)],
                        [np.sin(yaw), np.cos(yaw)]])
        rot_offset = np.dot(rot, offset)
        lidar_xy = vehicle_xy + rot_offset

        self._lidar_position = np.append(lidar_xy, yaw)


    def scan_callback(self, msg):
        lidar_pos = self._lidar_position[:2]
        yaw = self.vehicle_state.yaw
        points = []
        angle = msg.angle_min

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = lidar_pos[0] + r * cos(angle + yaw)
                y = lidar_pos[1] + r * sin(angle + yaw)
                points.append([x, y])
            angle += msg.angle_increment

        # Publish the rays and points
        publish_lidar_rays(self.rays_publisher, lidar_pos, points)
        publish_lidar_points(self.point_cloud_publisher, points)

if __name__ == '__main__':
    lidar_visualizer = LidarVisualizer()
    rospy.spin()

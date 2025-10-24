#!/usr/bin/env python

"""
Author: Frank Jiang, Javier Cerna

Module with some commonly useful visualization functions.

Currently, contains vizualization functions for:
    1. Vehicles
    2. Trajectories
    3. Lidar Scans
"""

from rclpy.clock import Clock
from rclpy.time import Time

import tf_transformations as tf
from geometry_msgs.msg import Point, Point32, PolygonStamped, PointStamped
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from svea_core import rosonic as rx

top_height = 0.22 # [m] approx.

##########################
## LIDAR VIZUALIZATIONS ##
##########################

def publish_lidar_points(point_cloud_publisher, points):
    """Publishes the detected points from a single channel Lidar as a
    pointcloud

    :param point_cloud_publisher: ROS publisher for broadcasting
                                  pointcloud with
    :type point_cloud_publisher: rospy.topics.Publisher
    :param points: List of detected points by lidar
    :type points: list
    """
    point_cloud = PointCloud()

    point_cloud.header.stamp = Clock().now().to_msg()
    point_cloud.header.frame_id = 'map'

    point_cloud.points = [Point32(x=point[0], y=point[1], z=top_height)
                          for point in points]

    point_cloud_publisher.publish(point_cloud)

def publish_lidar_rays(rays_publisher, lidar_pos, points):
    """Publishes the rays from a single channel Lidar corresponding to
    detected points (i.e. points in range)

    :param rays_publisher: ROS publisher for broadcasting rays with
    :type rays_publisher: rospy.topics.Publisher
    :param lidar_pos: Position of lidar, first two values should be XY
    :type lidar_pos: list
    :param points: List of detected points by lidar
    :type points: list
    """
    marker_msg = Marker()
    marker_msg.header.stamp = Clock().now().to_msg()
    marker_msg.header.frame_id = 'map'

    marker_msg.type = Marker.LINE_LIST
    marker_msg.action = Marker.ADD

    marker_msg.points = []

    for point in points:
        marker_msg.points.append(Point(x=lidar_pos[0],
                                       y=lidar_pos[1],
                                       z=top_height))
        marker_msg.points.append(Point(x=point[0],
                                       y=point[1],
                                       z=top_height))

    marker_msg.id = 0
    marker_msg.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.05)
    marker_msg.scale.x = 0.05
    marker_msg.pose.orientation.w = 1.0

    rays_publisher.publish(marker_msg)

def publish_edges(edges_publisher, edges):
    """Publishes the list of edges to RVIZ

    :param edges_publisher: ROS publisher for broadcasting edges with
    :type edges_publisher: rospy.topics.Publisher
    :param edges: List of edges to visualize
    :type edges: list
    """
    marker_msg = Marker()
    marker_msg.header.stamp = Clock().now().to_msg()
    marker_msg.header.frame_id = 'map'

    marker_msg.type = Marker.LINE_LIST
    marker_msg.action = Marker.ADD

    marker_msg.points = []

    for edge in edges:
        pt1 = edge[0]
        pt2 = edge[1]
        marker_msg.points.append(Point(x=pt1[0],
                                       y=pt1[1],
                                       z=top_height))
        marker_msg.points.append(Point(x=pt2[0],
                                       y=pt2[1],
                                       z=top_height))

    marker_msg.id = 0
    marker_msg.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
    marker_msg.scale.x = 0.05
    marker_msg.pose.orientation.w = 1.0

    edges_publisher.publish(marker_msg)

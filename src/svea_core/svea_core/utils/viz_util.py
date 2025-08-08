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
<<<<<<< HEAD
<<<<<<< HEAD
from svea_core import rosonic as rx

top_height = 0.22 # [m] approx.

=======
=======
from svea_core import rosonic as rx
>>>>>>> 9278f8e (Planned Path and Past Path can be displayed with ShowPath in util, example in pure pursuit)

top_height = 0.22 # [m] approx.

<<<<<<< HEAD
###############################
## TRAJECTORY VISUALIZATIONS ##
###############################

def lists_to_pose_stampeds(x_list, y_list, yaw_list=None, t_list=None):
    poses = []
    for i in range(len(x_list)):
        x = x_list[i]
        y = y_list[i]

        curr_pose = PoseStamped()
        curr_pose.header.frame_id = 'map'
        curr_pose.pose.position.x = x
        curr_pose.pose.position.y = y

        if not yaw_list is None:
            yaw = yaw_list[i]
            quat = tf.quaternion_from_euler(0.0, 0.0, yaw)
            curr_pose.pose.orientation.x = quat[0]
            curr_pose.pose.orientation.y = quat[1]
            curr_pose.pose.orientation.z = quat[2]
            curr_pose.pose.orientation.w = quat[3]

        if not t_list is None:
            t = t_list[i]
            curr_pose.header.stamp = Time(secs = t).to_msg()
        else:
            curr_pose.header.stamp = Clock().now().to_msg()

        poses.append(curr_pose)
    return poses

def lists_to_poses(x_list, y_list, yaw_list=None):
    poses = []
    for i in range(len(x_list)):
        x = x_list[i]
        y = y_list[i]

        curr_pose = Pose()
        curr_pose.position.x = x
        curr_pose.position.y = y

        if not yaw_list is None:
            yaw = yaw_list[i]
            quat = tf.quaternion_from_euler(0.0, 0.0, yaw)
            curr_pose.orientation.x = quat[0]
            curr_pose.orientation.y = quat[1]
            curr_pose.orientation.z = quat[2]
            curr_pose.orientation.w = quat[3]

        poses.append(curr_pose)
    return poses

def publish_path(path_publisher, x_list, y_list, yaw_list=None, t_list=None):
    """Publish trajectory visualization to rviz
    *t_list untested

    :param path_publisher: ROS2 publisher to broadcast trajectory with
    :type path_publisher: rclpy.topics.Publisher
    :param x_list: x trajectory in [m]
    :type x_list: list
    :param y_list: y trajecotory in [m]
    :type y_list: list
    :param yaw_list: yaw trajectory in [rad], defaults to None
    :type yaw_list: list
    :param t_list: time trajectory in [s], defaults to None
    :type t_list: list
    """

    path = Path()
    path.header.stamp = Clock().now().to_msg()
    path.header.frame_id = 'map'

    path.poses = lists_to_pose_stampeds(x_list, y_list, yaw_list, t_list)
    path_publisher.publish(path)

def publish_target(target_publisher, x, y):
    """
    Publish target point for visualization in rviz

    :param target_publisher: ROS publisher to broadcast target with
    :type target_publisher: rospy.topics.Publisher
    """
    target_pt = PointStamped()
    target_pt.header.stamp = Clock().now().to_msg()
    target_pt.header.frame_id = 'map'
    target_pt.point.x = x
    target_pt.point.y = y
    target_pt.point.z = chassis_height
    target_publisher.publish(target_pt)

def publish_pose_array(poses_publisher, x_list, y_list, yaw_list):
    """Publish list of poses for visualization to rviz

    :param poses_publisher: ROS publisher to broadcast trajectory with
    :type poses_publisher: rospy.topics.Publisher
    :param x_list: x coordinates in [m]
    :type x_list: list
    :param y_list: y coordinates in [m]
    :type y_list: list
    :param yaw_list: yaw coordinates in [rad]
    :type yaw_list: list
    """
    pose_array = PoseArray()
    pose_array.header.stamp = Clock().now().to_msg()
    pose_array.header.frame_id = 'map'
    pose_array.poses = lists_to_poses(x_list, y_list, yaw_list)
    poses_publisher.publish(pose_array)


>>>>>>> 146db9c (marker placer complete)
=======
>>>>>>> 9278f8e (Planned Path and Past Path can be displayed with ShowPath in util, example in pure pursuit)
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

#!/usr/bin/env python

"""
Module with some commonly useful visualization functions.

Currently, contains vizualization functions for:
    1. Vehicles
    2. Trajectories
    3. Lidar Scans
"""

import numpy as np
import math
import matplotlib.pyplot as plt
import rospy

import tf
from geometry_msgs.msg import Point, Point32, PolygonStamped, PointStamped
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

__license__ = "MIT"
__maintainer__ = "Frank Jiang, Javier Cerna"
__email__ = "frankji@kth.se"
__status__ = "Development"


############################
## VEHICLE VIZUALIZATIONS ##
############################

# SVEA Vehicle parameters
LENGTH = 0.586  # [m]
WIDTH = 0.2485  # [m]
BACKTOWHEEL = 0.16  # [m]
WHEEL_LEN = 0.03  # [m]
WHEEL_WIDTH = 0.02  # [m]
TREAD = 0.07  # [m]
WB = 0.324  # [m]

chassis_height = 0.06 # [m] approx.
top_height = 0.22 # [m] approx.


def plot_car(x, y, yaw, steer=0.0, color="-k"):
    """
    Plotting function from PythonRobotics MPC

    :param x: Current x position of car in [m]
    :type x: float
    :param y: Current y position of car in [m]
    :type y: float
    :param yaw: Current yaw of car in [rad]
    :type yaw: float
    :param steer: Current steering angle of car's front wheels [rad]
    :type steer: float
    :param color: Color of plotted vehicle works with matplotlib colors
    :type color: str
    """

    outline = np.matrix([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                         [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.matrix([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                          [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.matrix([[math.cos(yaw), math.sin(yaw)],
                      [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.matrix([[math.cos(steer), math.sin(steer)],
                      [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T * Rot2).T
    fl_wheel = (fl_wheel.T * Rot2).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T * Rot1).T
    fl_wheel = (fl_wheel.T * Rot1).T

    outline = (outline.T * Rot1).T
    rr_wheel = (rr_wheel.T * Rot1).T
    rl_wheel = (rl_wheel.T * Rot1).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), color)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), color)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), color)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), color)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), color)
    plt.plot(x, y, "*")

def publish_3Dcar(polygon_publisher, pose_publisher, x, y, yaw):
    """
    Publish 3D car polygon and fake odometry pose to rviz. More
    customization to 3D car appearance in RVIZ configuration

    :param polygon_publisher: ROS publisher to broadcast polygon with
    :type polygon_publisher: rospy.topics.Publisher
    :param pose_publisher: ROS publisher to broadcast fake pose with
    :type pose_publisher: rospy.topics.Publisher
    :param x: current x position of car in [m]
    :type x: float
    :param y: current y position of car in [m]
    :type y: float
    :param yaw: current yaw of car in [rad]
    :type yaw: float
    """

    outline = np.matrix([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL),
                          (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                         [WIDTH / 2, WIDTH / 2, - WIDTH / 2,
                          -WIDTH / 2, WIDTH / 2]])

    Rot1 = np.matrix([[math.cos(yaw), math.sin(yaw)],
                      [-math.sin(yaw), math.cos(yaw)]])

    outline = (outline.T * Rot1).T

    outline[0, :] += x
    outline[1, :] += y

    # more messy than need be
    xs = np.array(outline[0, :]).flatten().tolist()
    ys = np.array(outline[1, :]).flatten().tolist()

    pts = [Point32(xs[0], ys[0], chassis_height),
           Point32(xs[1], ys[1], chassis_height),
           Point32(xs[2], ys[2], chassis_height),
           Point32(xs[3], ys[3], chassis_height),
           Point32(xs[0], ys[0], chassis_height),
           Point32(xs[0], ys[0], top_height),
           Point32(xs[1], ys[1], top_height),
           Point32(xs[1], ys[1], chassis_height),
           Point32(xs[1], ys[1], top_height),
           Point32(xs[2], ys[2], top_height),
           Point32(xs[2], ys[2], chassis_height),
           Point32(xs[2], ys[2], top_height),
           Point32(xs[3], ys[3], top_height),
           Point32(xs[3], ys[3], chassis_height),
           Point32(xs[3], ys[3], top_height),
           Point32(xs[0], ys[0], top_height)
           ]

    # build polygon
    car_poly = PolygonStamped()
    car_poly.header.stamp = rospy.Time.now()
    car_poly.header.frame_id = 'map'
    car_poly.polygon.points = pts

    # load odometry
    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)

    car_pose = PoseStamped()
    car_pose.header.stamp = rospy.Time.now()
    car_pose.header.frame_id = 'map'
    car_pose.pose.position.x = x
    car_pose.pose.position.y = y
    car_pose.pose.orientation.x = quat[0]
    car_pose.pose.orientation.y = quat[1]
    car_pose.pose.orientation.z = quat[2]
    car_pose.pose.orientation.w = quat[3]

    polygon_publisher.publish(car_poly)
    pose_publisher.publish(car_pose)


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
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
            curr_pose.pose.orientation.x = quat[0]
            curr_pose.pose.orientation.y = quat[1]
            curr_pose.pose.orientation.z = quat[2]
            curr_pose.pose.orientation.w = quat[3]

        if not t_list is None:
            t = t_list[i]
            curr_pose.header.stamp = rospy.Time(secs = t)
        else:
            curr_pose.header.stamp = rospy.Time.now()

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
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
            curr_pose.orientation.x = quat[0]
            curr_pose.orientation.y = quat[1]
            curr_pose.orientation.z = quat[2]
            curr_pose.orientation.w = quat[3]

        poses.append(curr_pose)
    return poses

def publish_path(path_publisher, x_list, y_list, yaw_list=None, t_list=None):
    """Publish trajectory visualization to rviz
    *t_list untested

    :param path_publisher: ROS publisher to broadcast trajectory with
    :type path_publisher: rospy.topics.Publisher
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
    path.header.stamp = rospy.Time.now()
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
    target_pt.header.stamp = rospy.Time.now()
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
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.frame_id = 'map'
    pose_array.poses = lists_to_poses(x_list, y_list, yaw_list)
    poses_publisher.publish(pose_array)


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

    point_cloud.header.stamp = rospy.Time.now()
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
    marker_msg.header.stamp = rospy.Time.now()
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
    marker_msg.color = ColorRGBA(r=1, g=0, b=0, a=0.05)
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
    marker_msg.header.stamp = rospy.Time.now()
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
    marker_msg.color = ColorRGBA(r=0, g=0, b=1, a=1)
    marker_msg.scale.x = 0.05
    marker_msg.pose.orientation.w = 1.0

    edges_publisher.publish(marker_msg)

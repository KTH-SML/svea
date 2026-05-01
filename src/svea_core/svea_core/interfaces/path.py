#!/usr/bin/env python

from collections import deque

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from rclpy.clock import Clock
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import tf_transformations as tf

from .. import rosonic as rx

qos_subber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable
    durability=QoSDurabilityPolicy.VOLATILE,    # Volatile
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep the last N messages
    depth=10,                                   # Size of the queue
)

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

class ShowPath(rx.Field):
    """
    TODO:
        - Build more general interface for navigation (with nav2 topics?)
    """
    
    PATH_MAX_LEN = 50

    odometry_top = rx.Parameter('odometry/local')

    past_path_pub = rx.Publisher(Path, 'past_path', qos_pubber)
    path_plan_pub = rx.Publisher(Path, 'path_plan', qos_pubber)

    @rx.Subscriber(Odometry, odometry_top, qos_subber)
    def odometry_callback(self, msg):
        self.past_path.append(self._odom_to_pose(msg))
        path = Path()
        path.header = msg.header
        path.poses = self.past_path
        self.past_path_pub.publish(path)
    
    def __init__(self):

        self.past_path = deque(maxlen=self.PATH_MAX_LEN)

    def publish_path(self, x_list, y_list, yaw_list=None, t_list=None):
        """Publish trajectory visualization

        Args:
            x_list: x trajectory in [m]
            y_list: y trajecotory in [m]
            yaw_list: yaw trajectory in [rad], defaults to None
            t_list: time trajectory in [s], defaults to None
        """

        path = Path()
        path.header.stamp = Clock().now().to_msg()
        path.header.frame_id = 'map'

        poses = []
        for i, (x, y) in enumerate(zip(x_list, y_list)):

            curr_pose = PoseStamped()
            curr_pose.header.frame_id = path.header.frame_id
            curr_pose.pose.position.x = x
            curr_pose.pose.position.y = y

            if yaw_list is not None:
                yaw = yaw_list[i]
                quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
                curr_pose.pose.orientation.x = quat[0]
                curr_pose.pose.orientation.y = quat[1]
                curr_pose.pose.orientation.z = quat[2]
                curr_pose.pose.orientation.w = quat[3]

            if t_list is not None:
                t = t_list[i]
                curr_pose.header.stamp = Time(secs = t).to_msg()
            else:
                curr_pose.header.stamp = path.header.stamp

            poses.append(curr_pose)

        path.poses = poses
        self.path_plan_pub.publish(path)

    def _odom_to_pose(self, msg):
        """Odometry -> PoseStamped"""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        return pose


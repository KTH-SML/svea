#!/usr/bin/env python
from .. import rosonic as rx
from matplotlib.colors import to_rgba, is_color_like
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point, Point32, PolygonStamped, PointStamped
from rclpy.clock import Clock
import tf_transformations as tf
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from rclpy.time import Time
from collections import deque

qos_subber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep the last N messages
    durability=QoSDurabilityPolicy.VOLATILE,    # Volatile
    depth=10,                                   # Size of the queue
)

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

class ShowPath(rx.Field):
    
    TRAVEL_DIST_THRESH = 0.1
<<<<<<< HEAD
    PATH_MAX_LEN = 100
=======
    PATH_MAX_LEN = 1000
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
    chassis_height=0.06

    odometry_top = rx.Parameter('odometry/local')

    past_path_pub = rx.Publisher(Path,'past_path',qos_pubber)
    path_pub = rx.Publisher(Path, 'path_plan',qos_pubber)

    @rx.Subscriber(Odometry, odometry_top, qos_subber)
    def odometry_callback(self, msg):
        new_path = self.odom_to_pose_stampeds(msg)
        self.past_path += new_path
        path = Path()
        path.header = msg.header
        path.poses = self.past_path
        self.past_path_pub.publish(path)
    
    def __ini__(self):

        self.past_path = deque(maxlen=self.PATH_MAX_LEN)
    
    def on_startup(self):

        self.traj_x = []
        self.traj_y = []
        self.x = []
        self.y = []
        self.past_path= []

    def odom_to_pose_stampeds(self, msg):
        poses = []

        curr_pose = PoseStamped()
        curr_pose.header = msg.header
        curr_pose.pose = msg.pose.pose
        poses.append(curr_pose)
        return poses
    
    def lists_to_pose_stampeds(self, x_list, y_list, yaw_list=None, t_list=None):
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
                curr_pose.header.stamp = Time(secs = t).to_msg()
            else:
                curr_pose.header.stamp = Clock().now().to_msg()

            poses.append(curr_pose)
        return poses

    def lists_to_poses(self, x_list, y_list, yaw_list=None):
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


    def publish_path(self, x_list, y_list, yaw_list=None, t_list=None):
        """Publish trajectory visualization to rviz
        *t_list untested

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

        path.poses = self.lists_to_pose_stampeds(x_list, y_list, yaw_list, t_list)
        self.path_pub.publish(path)

    def publish_target(self, target_publisher, x, y):
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
        target_pt.point.z = self.chassis_height
        target_publisher.publish(target_pt)

    def publish_pose_array(self, poses_publisher, x_list, y_list, yaw_list):
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
        pose_array.poses = self.lists_to_poses(x_list, y_list, yaw_list)
        poses_publisher.publish(pose_array)
#! /usr/bin/env python3

import rclpy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import math
import rclpy.clock
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf_transformations import quaternion_from_euler
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

        

class PlannerInterface(object):
    _path_interface = None
    _gridmap_msg = None
    _gridmap_sub = None
    _map_topic = None
    _gridmap = None
    _path_pub = None
    _path_topic = None
    _points_path = None

    def __init__(self,Node):
        self.node = Node
        # Map interface
        self._map_topic = self.load_param('~map_topic', '/map')
        self.init_gridmap_subscribers()

        # Path interface
        self._path_topic = self.load_param('~path_topic', '/path')
        self._path_pub = self.node.create_publisher(Path, self._path_topic, 1)
        # self._path_pub = rclpy.Publisher(self._path_topic, Path, latch=True, queue_size=1)
        self._pose_path = list()
        self._rviz_path = Path()
        self.init_gridmap_subscribers()
        self.ndoe.get_logger().info("PlannerInterface initialized")
    
    def load_param(self, name, value=None):
        self.node.declare_parameter(name, value)
        if value is None:
            assert self.node.has_parameter(name), f'Missing parameter "{name}"'
        return self.node.get_parameter(name).value
    
    ## GRIDMAP INTERFACE ##
    def init_gridmap_subscribers(self):
        self._gridmap_sub = self.node.create_subscription(OccupancyGrid, self._map_topic, self._gridmap_cb, 1)
        
    def _gridmap_cb(self, msg):
        self._gridmap_msg = OccupancyGrid(msg.header, msg.info, msg.data)

    def _get_delta(self):
        if self._gridmap_msg  is not None:
            return [self._gridmap_msg.info.resolution, self._gridmap_msg.info.resolution]
    
    def _get_limits(self):
        if self._gridmap_msg is not None:
            return [[0, self._gridmap_msg.info.width * self._gridmap_msg.info.resolution], [0, self._gridmap_msg.info.height * self._gridmap_msg.info.resolution]]
    
    def _get_obstacles(self):
        if self._gridmap_msg  is not None:
            gridmap = np.array(self._gridmap_msg.data).reshape((self._gridmap_msg.info.height, self._gridmap_msg.info.width)).T * 0.01
            # Cell values are changed to -0.01 (unknown), 0 (free), 1 (obstacle)
            obstacles = []
            for (x, y), cell in np.ndenumerate(gridmap):
                #!! Strong assumption: unknown cells are considered as obstacles
                if cell < 0 or cell == 1:
                    obstacles.append([x, y, self._gridmap_msg.info.resolution])
            return obstacles

    def get_planner_world(self):
        while self._gridmap_msg is None:
            continue
        return self._get_delta(), self._get_limits(), self._get_obstacles()

    def publish_map_internal_representation(self):
        obstacles = []
        delta, limits, obs = self.get_planner_world()

        rows_int = int(np.round(limits[0][1] / delta[0]))
        cols_int = int(np.round(limits[1][1] / delta[1]))
        data = np.zeros(rows_int * cols_int).reshape(rows_int, cols_int)
        [obstacles.append(tup[0:2]) for tup in obs]
        obs_np = np.asarray(obstacles)
        data[obs_np[:, 0], obs_np[:, 1]] = 100  

        map_pub = self.node.create_publisher(OccupancyGrid, self._map_topic, 1)
        # map_pub = rclpy.Publisher('/map_from_grid', OccupancyGrid, latch=True, queue_size=1)
        map_msg = OccupancyGrid()
        map_msg.data = [item for sublist in np.array(data, dtype=int).tolist() for item in sublist]
        map_msg.header.frame_id = 'map'
        map_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        map_msg.info.resolution = delta[1]
        map_msg.info.width = cols_int
        map_msg.info.height = rows_int
        quat = quaternion_from_euler(0, math.pi , -math.pi / 2)
        map_msg.info.origin.orientation.x = quat[0]
        map_msg.info.origin.orientation.y = quat[1]
        map_msg.info.origin.orientation.z = quat[2]
        map_msg.info.origin.orientation.w = quat[3]

        print('Publishing map...')
        map_pub.publish(map_msg)
    ## END OF GRIDMAP INTERFACE##

    ## PATH INTERFACE ##
    def initialize_path_interface(self):
        self._path_topic = self.load_param('~path_topic', '/path')
        self.latch_qos = QoSProfile(depth=1,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._path_pub = self.node.create_publisher(Path, self._path_topic, self.latch_qos, 1)

    def create_pose_path(self):
        for point in self._points_path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rclpy.clock.Clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0
            self._pose_path.append(pose)

    def publish_rviz_path(self):
        if self._pose_path:
            self._rviz_path.header.frame_id = 'map'
            self._rviz_path.header.stamp = rclpy.clock.Clock().now().to_msg()
            self._rviz_path.poses = self._pose_path
            print('Publishing path (length = {}) ...'.format(len(self._pose_path)))
            self._path_pub.publish(self._rviz_path)

    def set_points_path(self, path):
        self._points_path = path
    
    def get_points_path_reduced(self, granularity=4):
        indexes = np.linspace(0, len(self._points_path) - 1, num=int(np.round(len(self._points_path) / granularity)), dtype=int).tolist()
        return np.array(self._points_path)[indexes]
    ## END OF PATH INTERFACE ##

    # def get_path_from_topic(self):
    #     msg = rospy.wait_for_message('/smooth_path', Path, timeout=None)
    #     path = []
    #     for p in msg.poses:
    #         path.append([p.pose.position.x, p.pose.position.y])
    #     return path
    
    def get_path_from_topic(self):
        path_msg = self.node.create_subscription(
            Path,
            '/smooth_path',
            lambda msg: setattr(self, '_temp_path_msg', msg),
            10
        )
        
        # Wait until we receive a message
        while not hasattr(self, '_temp_path_msg'):
            rclpy.spin_once(self)
        
        # Process the message
        path = []
        for p in self._temp_path_msg.poses:
            path.append([p.pose.position.x, p.pose.position.y])
        
        # Clean up
        del self._temp_path_msg
        self.node.destroy_subscription(path_msg)
        
        return path
        
    def publish_path(self):
        self.create_pose_path()
        self.publish_rviz_path()

    def get_points_path(self, granularity=None):
        """
        Function to get every (x, y) point composing the path

        :param granularity: get one point of the path each N-granularity points, defaults to None
        :type granularity: integer, optional
        :param interpolate: choose if path smoother has to interplate or approximate path, defaults to False
        :type iterpolate: boolean
        :return: list of points
        :rtype: list[float]
        """
        if granularity is not None:
            path = self.get_points_path_reduced(granularity)
        else:
            path = self._points_path
        return path
          
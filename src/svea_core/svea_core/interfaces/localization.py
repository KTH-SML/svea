"""
Author: Tobias Bolin, Frank Jiang
"""

from typing import Callable, Self

import rclpy
from rclpy.node import Node
import rclpy.clock
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from tf_transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry


__all__ = [
    'LocalizationInterface',
]

class LocalizationInterface[N:Node]:
    """Interface handling the reception of state information from the
    localization stack.

    This object can take on several callback functions and execute them as soon
    as state information is available.

    Args:
        vehicle_name: Name of vehicle being controlled; The name will be
            effectively be added as a namespace to the topics used by the
            corresponding localization node i.e `namespace/vehicle_name/state`.
    """

    def __init__(self, node: N)-> None:
        
        self.node = node
        self._odom_topic = 'odometry/local'

        self.last_time = float('nan')

        # list of functions to call whenever a new state comes in
        self.callbacks = []

    def start(self) -> Self:
        """Spins up ROS background thread; must be called to start receiving
        data.
        """
        self.node.get_logger().info("Starting Localization Interface Node...")
        self.node_name = 'localization_node'
        self._start_listen()

        self.node.get_logger().info("Localization Interface is ready")
        return self

    def _start_listen(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.node.create_subscription(Odometry, self._odom_topic,
                                 self.odom_callback, qos_profile)
        
    def odom_callback(self, msg):
        self.last_odom = msg
        for cb in self.callbacks:
            try:
                cb(msg)
            except Exception as e:
                self.node.get_logger().error(f"Error in callback: {e}")

    def add_callback(self, cb: Callable[[N], None])->None:
        """Add state callback.

        Every function passed into this method will be called whenever new
        state information comes in from the localization stack.

        Args:
            cb: A callback function intended for responding to the reception of
                state info.
        """
        self.callbacks.append(cb)

    def remove_callback(self, cb: Callable[[N], None])->None:
        """Remove callback so it will no longer be called when state
        information is received.

        Args:
            cb: A callback function that should be no longer used in response
            to the reception of state info.
        """
        while cb in self.callbacks:
            self.callbacks.pop(self.callbacks.index(cb))

    @staticmethod
    def new_state(x,y,yaw,vel,stamp=None,frame_id='map',child_frame='base_link'):
        msg = Odometry()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        msg.twist.twist.linear.x = vel
        msg.header.stamp = (stamp if stamp is not None else 
                            rclpy.clock.Clock().now().to_msg())
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame
        return msg

    def get_state(self):
        return (LocalizationInterface.get_x(self.last_odom),
                LocalizationInterface.get_y(self.last_odom), 
                LocalizationInterface.get_yaw(self.last_odom), 
                LocalizationInterface.get_vel(self.last_odom))

     
    def get_x(self):
        """ Extract the x position from a odpmetry message
        :param msg: An Odometry message
        :return: The x position
        :rtype: float
        """
        return self.last_odom.pose.pose.position.x

     
    def get_y(self):
        """ Extract the y position from a odpmetry message
        :param msg: An Odometry message
        :return: The y position
        :rtype: float
        """
        return self.last_odom.pose.pose.position.y

     
    def get_yaw(self):
        """ Extract the yaw from a odpmetry message
        :param msg: An Odometry message
        :return: The yaw
        :rtype: float
        """
        quat = [self.last_odom.pose.pose.orientation.x,
                self.last_odom.pose.pose.orientation.y,
                self.last_odom.pose.pose.orientation.z,
                self.last_odom.pose.pose.orientation.w]
        return euler_from_quaternion(quat)[2]

     
    def get_vel(self):
        """ Extract the velocity from a odpmetry message
        :param msg: An Odometry message
        :return: The velocity
        :rtype: float
        """
        return self.last_odom.twist.twist.linear.x

    def set_x(self, x):
        """ Set the x position in an Odometry message
        :param msg: An Odometry message
        :param x: The new x position
        """
        self.last_odom.pose.pose.position.x = x

    def set_y(self, y):
        """ Set the y position in an Odometry message
        :param msg: An Odometry message
        :param y: The new y position
        """
        self.last_odom.pose.pose.position.y = y

    def set_yaw(self, yaw):
        """ Set the yaw in an Odometry message
        :param msg: An Odometry message
        :param yaw: The new yaw
        """
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        self.last_odom.pose.pose.orientation.x = quat[0]
        self.last_odom.pose.pose.orientation.y = quat[1]
        self.last_odom.pose.pose.orientation.z = quat[2]
        self.last_odom.pose.pose.orientation.w = quat[3]

    def set_vel(self, vel):
        """ Set the velocity in an Odometry message
        :param msg: An Odometry message
        :param vel: The new velocity
        """
        self.last_odom.twist.twist.linear.x = vel

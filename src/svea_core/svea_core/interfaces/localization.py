"""
Author: Tobias Bolin, Frank Jiang
"""

from typing import Self, Optional

import rclpy
from rclpy.node import Node
import rclpy.clock
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from tf_transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from .. import rosonic as rx
import time

qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)



class LocalizationInterface(rx.Resource):
    """Interface handling the reception of state information from the
    localization stack.

    This object can take on several callback functions and execute them as soon
    as state information is available.

    """

    _odom_top = 'odometry/local'

    @rx.Subscriber(Odometry, _odom_top, qos_profile=qos_profile)
    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_msg = msg
        for cb in self._odom_callbacks:
            try:
                cb(msg)
            except Exception as e:
                self.node.get_logger().error(f"Error in callback: {e}")


    def __init__(self, *, init_odom=None, **kwds) -> None:
        
        ## Odometry ##
        # Create a new odometry message with default values
        if odom := kwds.get('init_odom', None):
            self._odom_msg = init_odom
        else:
            self._odom_msg = Odometry()
            self._odom_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
            self._odom_msg.header.frame_id = 'map'
            self._odom_msg.child_frame_id = 'base_link'
        
        if odom_top := kwds.get('odom_top', None):
            self._odom_top = odom_top
        
        # list of functions to call whenever a new state comes in
        self._odom_callbacks = []


    def on_startup(self):
        """Start the localization interface by subscribing to the odometry topic."""
        self.node.get_logger().info("Starting Localization Interface Node...")

        time.sleep(10)  # Allow time for the node to initialize

        self.node.get_logger().info("Localization Interface is ready.")
        return self
        

    def add_callback(self, cb, as_state=False) -> None:
        """Add state callback.

        Every function passed into this method will be called whenever new
        state information comes in from the localization stack.

        Args:
            cb: A callback function intended for responding to the reception of
                state info.
            as_state: If True, the callback will be called with the state tuple
                (x, y, yaw, vel) instead of the odometry message.
        """
        if as_state:
            cb = lambda msg: cb(self.get_state(msg))
        self._odom_callbacks.append(cb)

    def remove_callback(self, cb) -> None:
        """Remove callback so it will no longer be called when state
        information is received.

        Args:
            cb: A callback function that should be no longer used in response
            to the reception of state info.
        """
        while cb in self._odom_callbacks:
            self._odom_callbacks.pop(self._odom_callbacks.index(cb))

    def get_state(self, odom=None) -> tuple[float, float, float, float]:
        """Get the current state of the localization interface.
        
        Args:
            odom: An optional odometry message to use instead of the last
                received message.

        Returns:
            A tuple containing the x position, y position, yaw angle, and
            velocity of the robot.
        """

        if odom is None:
            odom = self._odom_msg

        return (self.get_x(odom), self.get_y(odom), self.get_yaw(odom), self.get_vel(odom))
     
    def get_x(self, odom=None) -> float:
        """ Extract the x position from a odpmetry message

        Args:
            odom: An optional odometry message to use instead of the last
            received message.

        Returns:
            The x position.
        """
        if odom is None:
            odom = self._odom_msg
        return odom.pose.pose.position.x

    def get_y(self, odom=None) -> float:
        """ Extract the y position from a odpmetry message
        Args:
            odom: An optional odometry message to use instead of the last
            received message.
        Returns:
            The y position.
        """
        if odom is None:
            odom = self._odom_msg
        return odom.pose.pose.position.y
     
    def get_yaw(self, odom=None) -> float:
        """ Extract the yaw from a odpmetry message
        Args:
            odom: An optional odometry message to use instead of the last
            received message.
        Returns:
            The yaw angle in radians.
        """
        if odom is None:
            odom = self._odom_msg
        quat = [odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w]
        return euler_from_quaternion(quat)[2]

    def get_vel(self, odom=None) -> float:
        """ Extract the velocity from a odpmetry message
        Args:
            odom: An optional odometry message to use instead of the last
            received message.
        Returns:
            The velocity in m/s.
        """
        if odom is None:
            odom = self._odom_msg
        return odom.twist.twist.linear.x
"""
Author: Tobias Bolin, Frank Jiang
"""

from typing import Callable, Self, Optional

import rclpy
from rclpy.node import Node
import rclpy.clock
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from tf_transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry


__all__ = [
    'LocalizationInterface',
]

def wait_for_message(node, topic, msg_type, timeout=None):
    future = rclpy.task.Future()
    sub = node.create_subscription(
        msg_type,
        topic,
        lambda msg: future.set_result(msg),
        10
    )
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    node.destroy_subscription(sub)
    return future.result()

class LocalizationInterface:
    """Interface handling the reception of state information from the
    localization stack.

    This object can take on several callback functions and execute them as soon
    as state information is available.

    """

    _odom_top = 'odometry/local'

    def __init__(self, node: Node, *, init_odom=None, **kwds)-> None:
        
        self._node = node

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

    def start(self, wait=True) -> Self:
        
        self._node.get_logger().info("Starting Localization Interface Node...")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self._node.create_subscription(Odometry, self._odom_top, self._odom_cb, qos_profile)

        if wait:
            self.wait()

        self._node.get_logger().info("Localization Interface is ready.")
        return self

    def wait(self, timeout: Optional[float] = None) -> Odometry:
        """Wait for a message on the interface topic.

        Args:
            timeout: The time to wait for a message in seconds.
        """
        return wait_for_message(self._node, self._odom_top, Odometry, timeout=timeout)

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_msg = msg
        for cb in self._odom_callbacks:
            try:
                cb(msg)
            except Exception as e:
                self._node.get_logger().error(f"Error in callback: {e}")

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

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 54289ac (2025/04/16 Meeting Update)
"""
Author: Tobias Bolin, Frank Jiang
"""

<<<<<<< HEAD
from typing import Self, Optional
=======
from threading import Thread, Event
from typing import Callable
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)
=======
from typing import Callable, Self
>>>>>>> 54289ac (2025/04/16 Meeting Update)

import rclpy
from rclpy.node import Node
<<<<<<< HEAD
from rclpy.time import Time
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import tf2_ros
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

from tf2_geometry_msgs import do_transform_pose

from .. import rosonic as rx

=======
import rclpy.clock
<<<<<<< HEAD
<<<<<<< HEAD
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from tf_transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from .. import rosonic as rx
import time
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))

qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)


<<<<<<< HEAD
=======

>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
class LocalizationInterface(rx.Field):
    r"""Interface handling the reception of state information from the
=======
import rclpy.context
import rclpy.logging
=======
>>>>>>> 54289ac (2025/04/16 Meeting Update)
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from tf_transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry


__all__ = [
    'LocalizationInterface',
]

class LocalizationInterface[N:Node]:
    """Interface handling the reception of state information from the
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)
    localization stack.

    This object can take on several callback functions and execute them as soon
    as state information is available.
<<<<<<< HEAD
    """

    class _InterfaceParameters(rx.NamedField):
        odom_top = rx.Parameter('odometry/local')

    _params = _InterfaceParameters(name='localization')
    _odom_msg = Odometry()

    def __init__(self) -> None:
        # list of functions to call whenever a new state comes in
        self._odom_callbacks = []

    def on_startup(self):
        """Start the localization interface by subscribing to the odometry topic."""
        self.node.get_logger().info("Starting Localization interface Node...")

        # Transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node, spin_thread=True)
        
        self.node.get_logger().info("Localization interface is ready.")
        return self._is_started
        
    @rx.Subscriber(Odometry, _params.odom_top, qos_profile=qos_profile)
    def _odom_cb(self, msg: Odometry) -> None:
        if not self._is_started():
            return

        self._odom_msg = self.transform_odom(msg)
        for cb in self._odom_callbacks:
            try:
                cb(self._odom_msg)
            except Exception as e:
                self.node.get_logger().error(f"Error in callback: {e}")

    def transform_odom(
        self,
        odom: Odometry,
        pose_target: str = "map",
        twist_target: str = "base_link",
        timeout_s: float = 0.2,
    ) -> Odometry:
        """
        Transform an Odometry message so that:
          - pose is expressed in `pose_target` (default: 'map')
          - twist is expressed in `twist_target` (default: 'base_link')

        By spec:
          - pose source frame  = odom.header.frame_id
          - twist source frame = odom.child_frame_id
        """

        assert self._is_started(), 'Localization interface not started yet'

        # ---- Source frames from the incoming message ----
        pose_source  = odom.header.frame_id or ""
        twist_source = odom.child_frame_id or pose_source or ""

        stamp = Time()
        timeout = Duration(seconds=timeout_s)

        # ---- Transform Pose -> pose_target ----
        ps = PoseStamped()
        ps.header = odom.header
        ps.header.stamp = stamp.to_msg()
        ps.pose   = odom.pose.pose

        if pose_source != pose_target:
            # Guard (optional but nice for clearer logs)
            if not self.tf_buffer.can_transform(pose_target, pose_source, stamp, timeout):
                raise RuntimeError(
                    f"No TF from '{pose_source}' to '{pose_target}' at {stamp.to_msg()}"
                )
            ps_out = self.tf_buffer.transform(ps, pose_target, timeout=timeout)
        else:
            ps_out = ps  # already in target

        # ---- Transform Twist -> twist_target ----
        ts = TwistStamped()
        ts.header.stamp = stamp.to_msg()
        ts.header.frame_id = twist_source if twist_source else pose_source
        ts.twist = odom.twist.twist

        if ts.header.frame_id != twist_target:
            if not self.tf_buffer.can_transform(twist_target, ts.header.frame_id, stamp, timeout):
                raise RuntimeError(
                    f"No TF from '{ts.header.frame_id}' to '{twist_target}' at {stamp.to_msg()}"
                )
            ts_out = self.tf_buffer.transform(ts, twist_target, timeout=timeout)
        else:
            ts_out = ts  # already in target

        # ---- Repack as Odometry: pose in pose_target, twist in twist_target ----
        out = Odometry()
        out.header.stamp = odom.header.stamp
        out.header.frame_id = pose_target             # pose frame
        out.child_frame_id  = twist_target            # twist (body) frame
        out.pose.pose = ps_out.pose
        out.twist.twist = ts_out.twist

        # NOTE: Covariances are copied verbatim. If you *need* correctness across frames,
        # rotate them (Σ' = R Σ Rᵀ) for the relevant 3x3 blocks. Otherwise, leave as-is.
        out.pose.covariance  = odom.pose.covariance
        out.twist.covariance = odom.twist.covariance

        return out

=======

<<<<<<< HEAD
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
        
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))

    def add_callback(self, cb, as_state=False) -> None:
=======
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

<<<<<<< HEAD
    def add_callback(self, cb: Callable[[VehicleState], None]):
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)
=======
    def add_callback(self, cb: Callable[[N], None])->None:
>>>>>>> 54289ac (2025/04/16 Meeting Update)
        """Add state callback.

        Every function passed into this method will be called whenever new
        state information comes in from the localization stack.

        Args:
            cb: A callback function intended for responding to the reception of
                state info.
<<<<<<< HEAD
            as_state: If True, the callback will be called with the state tuple
                (x, y, yaw, vel) instead of the odometry message.
        """
        if as_state:
            cb = lambda msg: cb(self.get_state(msg))
        self._odom_callbacks.append(cb)

    def remove_callback(self, cb) -> None:
=======
        """
        self.callbacks.append(cb)

<<<<<<< HEAD
    def remove_callback(self, cb: Callable[[VehicleState], None]):
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)
=======
    def remove_callback(self, cb: Callable[[N], None])->None:
>>>>>>> 54289ac (2025/04/16 Meeting Update)
        """Remove callback so it will no longer be called when state
        information is received.

        Args:
            cb: A callback function that should be no longer used in response
            to the reception of state info.
        """
<<<<<<< HEAD
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
<<<<<<< HEAD
<<<<<<< HEAD
        return odom.twist.twist.linear.x
=======
        return odom.twist.twist.linear.x
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
=======
        return odom.twist.twist.linear.x
=======
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
<<<<<<< HEAD
        msg.twist.twist.linear.x = vel
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)
<<<<<<< HEAD
>>>>>>> b921c25 (Added rmw-zenoh in dockerfile, added svea_example)
=======
=======
        self.last_odom.twist.twist.linear.x = vel
>>>>>>> 54289ac (2025/04/16 Meeting Update)
>>>>>>> bc58fab (2025/04/16 Meeting Update)

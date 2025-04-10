from threading import Thread, Event
from typing import Callable

import rclpy
from rclpy.node import Node
import rclpy.clock
import rclpy.context
import rclpy.logging
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from svea_core.states import VehicleState
from svea_msgs.msg import VehicleState as VehicleStateMsg

from tf_transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry


__license__ = "MIT"
__maintainer__ = "Tobias Bolin, Frank Jiang"
__email__ = "tbolin@kth.se "
__status__ = "Development"

__all__ = [
    'LocalizationInterface',
]

class LocalizationInterface(object):
    """Interface handling the reception of state information from the
    localization stack.

    This object can take on several callback functions and execute them as soon
    as state information is available.

    Args:
        vehicle_name: Name of vehicle being controlled; The name will be
            effectively be added as a namespace to the topics used by the
            corresponding localization node i.e `namespace/vehicle_name/state`.
    """

    def __init__(
        self,
        Node,
        vehicle_name: str = '',
    ):
        
        self.node = Node
        self.vehicle_name = vehicle_name
        sub_namespace = vehicle_name + '/' if vehicle_name else ''
        self._state_topic = sub_namespace + 'state'

        self.state = VehicleState()
        self.last_time = float('nan')

        self.is_ready = False
        self._ready_event = Event()
        rclpy.Context().on_shutdown(self._shutdown_callback)

        # list of functions to call whenever a new state comes in
        self.callbacks = []

    def start(self) -> 'LocalizationInterface':
        """Spins up ROS background thread; must be called to start receiving
        data.
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _wait_until_ready(self, timeout=20.0):
        tic= rclpy.clock.Clock().now().to_msg().sec
        self._ready_event.wait(timeout)
        toc = rclpy.clock.Clock().now().to_msg().sec
        wait = toc - tic
        return wait < timeout

    def _shutdown_callback(self):
        self._ready_event.set()


    def _init_and_spin_ros(self):
        self.node.get_logger().info("Starting Localization Interface Node for "
                                        + self.vehicle_name)
        self.node_name = 'localization_node'
        self._start_listen()
        self.is_ready = self._wait_until_ready()
        if not self.is_ready:
            self.node.get_logger().warn("Localization not responding during start of "
                                            "Localization Interface. Setting ready anyway.")
        self.is_ready = True
        self.node.get_logger().info("Localization Interface for {} is ready"
                                        .format(self.vehicle_name))
        rclpy.spin(self.node)

    def _start_listen(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.node.create_subscription(VehicleStateMsg, self._state_topic,
                                 self._read_state_msg, qos_profile)
        # rospy.Subscriber(self._state_topic,
        #                  VehicleStateMsg,
        #                  self._read_state_msg,
        #                  tcp_nodelay=True,
        #                  queue_size=1)

    def _read_state_msg(self, msg):
        self.state.state_msg = msg
        self.last_time = rclpy.clock.Clock().now().to_msg()
        self._ready_event.set()
        self._ready_event.clear()

        for cb in self.callbacks:
            cb(self.state)

    def add_callback(self, cb: Callable[[VehicleState], None]):
        """Add state callback.

        Every function passed into this method will be called whenever new
        state information comes in from the localization stack.

        Args:
            cb: A callback function intended for responding to the reception of
                state info.
        """
        self.callbacks.append(cb)

    def remove_callback(self, cb: Callable[[VehicleState], None]):
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

    @staticmethod
    def as_tuple(msg):
        return (State.get_x(msg), State.get_y(msg), State.get_yaw(msg), State.get_vel(msg))

    @staticmethod
    def get_x(msg):
        """ Extract the x position from a odpmetry message
        :param msg: An Odometry message
        :return: The x position
        :rtype: float
        """
        return msg.pose.pose.position.x

    @staticmethod
    def get_y(msg):
        """ Extract the y position from a odpmetry message
        :param msg: An Odometry message
        :return: The y position
        :rtype: float
        """
        return msg.pose.pose.position.y

    @staticmethod
    def get_yaw(msg):
        """ Extract the yaw from a odpmetry message
        :param msg: An Odometry message
        :return: The yaw
        :rtype: float
        """
        quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w]
        return euler_from_quaternion(quat)[2]

    @staticmethod
    def get_vel(msg):
        """ Extract the velocity from a odpmetry message
        :param msg: An Odometry message
        :return: The velocity
        :rtype: float
        """
        return msg.twist.twist.linear.x

    @staticmethod
    def set_x(msg, x):
        """ Set the x position in an Odometry message
        :param msg: An Odometry message
        :param x: The new x position
        """
        msg.pose.pose.position.x = x

    @staticmethod
    def set_y(msg, y):
        """ Set the y position in an Odometry message
        :param msg: An Odometry message
        :param y: The new y position
        """
        msg.pose.pose.position.y = y

    @staticmethod
    def set_yaw(msg, yaw):
        """ Set the yaw in an Odometry message
        :param msg: An Odometry message
        :param yaw: The new yaw
        """
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

    @staticmethod
    def set_vel(msg, vel):
        """ Set the velocity in an Odometry message
        :param msg: An Odometry message
        :param vel: The new velocity
        """
        msg.twist.twist.linear.x = vel

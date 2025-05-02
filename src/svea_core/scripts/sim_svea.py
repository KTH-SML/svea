"""
Simulation module for the SVEA platform. Creates fake ROS
subscriptions and publications that match the real car platform.
Intended for debugging code BEFORE running on a real car.

Author: Frank Jiang
"""

import math
import numpy as np
from threading import Thread

import rclpy
import rclpy.clock
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from svea_core import rosonic as rx
from svea_core.states import SVEAControlValues
from svea_core.models.bicycle import SimpleBicycleModel
from svea_msgs.msg import LLIControl as LLIControl
from svea_msgs.msg import LLIEmergency as LLIEmergency


qos_default = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep the last N messages
    depth=10,                                          # Size of the queue
    durability=QoSDurabilityPolicy.VOLATILE     # Volatile
)

class sim_svea(rx.Node):
    """
    Handles simulation of a SVEA vehicle. The object takes in a model
    and pretends to be the low-level of a SVEA vehicle. It will spin up
    a continuous loop that updates the state of the fake vehicle using
    the model dynamics at a fixed rate and takes control inputs by
    subscribing to the same low-level interface ROS topic the actual
    SVEA low-level system uses and even publishes actuated control to
    the same topic as the low-level system.
    """

    ## Constants ##

    LOC_PUB_FREQ = 50 # Hz
    CTRL_WAIT_TIME = 0.2 # s
    MAX_SPEED_0 = 1.7 # [m/s]
    MAX_SPEED_1 = 3.6 # [m/s]
    MAX_STEERING_ANGLE = 40*math.pi/180

    SPEED_NOISE_STD = 0.05
    STEER_NOISE_STD = 0.1

    # scaling factor, percentage to lli actuation
    PERC_TO_LLI_COEFF = 1.27

    ## Parameters ##

    time_step = rx.Parameter(0.02, 'Time step for simulation')
    publish_tf = rx.Parameter(True, 'Publish TF tree')

    state_top = rx.Parameter('state', 'State topic name')
    request_top = rx.Parameter('lli/ctrl_request', 'Control request topic name')
    actuated_top = rx.Parameter('lli/ctrl_actuated', 'Actuated control topic name')
    emergency_top = rx.Parameter('lli/emergency', 'Emergency topic name')
    odometry_top = rx.Parameter('odometry/local', 'Odometry topic name')

    map_frame = rx.Parameter('map', 'Map frame id')
    odom_frame = rx.Parameter('odom', 'Odom frame id')
    self_frame = rx.Parameter('base_link', 'Base link frame id')

    def on_startup(self):

        state = Odometry()
        state.header.frame_id = self.map_frame
        state.child_frame_id = self.self_frame
        
        self.model = SimpleBicycleModel(state)
        self.control_values = SVEAControlValues(0, 0, 0, False, False)

        self.clock = rclpy.clock.Clock()

        self.last_ctrl_time = self.clock.now().to_msg()
        self.last_pub_time = self.clock.now().to_msg()

        if self.publish_tf:
            # for broadcasting fake tf tree
            self.tf_br = tf2_ros.TransformBroadcaster(self)

        self.is_emergency = False
        self.sender_id = None

        ## Publishers ##
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        self.ctrl_actuated_pub = self.create_publisher(LLIControl,
                                                       self._actuated_topic,
                                                       qos_profile)

        self.odometry_pub = self.create_publisher(Odometry,
                                                  self._odometry_topic,
                                                  qos_profile)

        ## Subscribers ##
        
        self.create_subscription(LLIControl,
                                 self._request_topic,
                                 self._update_ctrl_request,
                                 10)
        self.create_subscription(LLIEmergency,
                                 self._emergency_topic,
                                 self._update_emergency,
                                 10)

        ## Timers ##

        self.create_timer(self.time_step, self.sim_loop)

    def sim_loop(self):
        curr_time = self.clock.now().to_msg()

        steering = self._percent_to_steer(self.control_values.steering / self.PERC_TO_LLI_COEFF)
        steering += np.random.normal(0, self.STEER_NOISE_STD)
        
        velocity = self._percent_to_vel(self.control_values.velocity / self.PERC_TO_LLI_COEFF)
        velocity += np.random.normal(0, self.SPEED_NOISE_STD)
        
        if not (curr_time.sec - self.last_ctrl_time.sec) < self.CTRL_WAIT_TIME:
            steering, velocity = 0.0, 0.0

        if self.is_emergency:
            velocity = 0

        self.model.update(steering, velocity, self.dt)

        # publish fake localization data
        if (curr_time.sec - self.last_pub_time.sec) > 1.0/self.LOC_PUB_FREQ:
            if self.publish_tf:
                self._broadcast_tf()

            self.odometry_pub.publish(self.model.state)
            self.last_pub_time = self.clock.now().to_msg()

    def _percent_to_steer(self, steering):
        """Convert radians to percent of max steering actuation"""
        steering = float(steering)
        steer_percent = steering/100*self.MAX_STEERING_ANGLE
        return steer_percent

    def _percent_to_vel(self, vel_percent):
        vel_percent = float(vel_percent)
        if self.control_values.gear == 0:
            velocity = vel_percent*self.MAX_SPEED_0 / 100
        elif self.control_values.gear == 1:
            velocity = vel_percent*self.MAX_SPEED_1 / 100
        return velocity

    def _broadcast_tf(self):
        map2odom = TransformStamped()
        map2odom.header.stamp = self.model.state.header.stamp
        map2odom.header.frame_id = self._map_frame_id
        map2odom.child_frame_id = self._odom_frame_id
        map2odom.transform.rotation.w = 1.0
        self.tf_br.sendTransform(map2odom)

        odom2base = TransformStamped()
        odom2base.header.stamp = self.model.state.header.stamp
        odom2base.header.frame_id = self._odom_frame_id
        odom2base.child_frame_id = self._base_link_frame_id
        pose = self.model.pose_msg.pose.pose
        odom2base.transform.translation.x = pose.position.x
        odom2base.transform.translation.y = pose.position.y
        odom2base.transform.translation.z = pose.position.z
        odom2base.transform.rotation = pose.orientation
        self.tf_br.sendTransform(odom2base)

    def _update_ctrl_request(self, ctrl_request_msg):
        self.last_ctrl_time = self.clock.now().to_msg()
        changed = self.control_values.update_from_msg(ctrl_request_msg)
        if changed:
            self.ctrl_actuated_pub.publish(self.control_values.ctrl_msg)

    @property
    def emergency(self):
        """Check if the emergency flag is set. Emulating actuation
        interface emergency check.

        :return: `True` if emergency flag is set, `False` otherwise,
                 `None` if no information has been received.
        :rtype: bool
        """
        return self.is_emergency

    def _update_emergency(self, emergency_msg):
        emergency = emergency_msg.emergency
        sender_id = emergency_msg.sender_id
        if self.is_emergency and (sender_id == self.sender_id):
            # only one who stated emergency allowed to change it
            if not emergency:
                # this means sender is calling not emergency
                self.is_emergency = False
                self.sender_id = None
        elif emergency:
            self.is_emergency = True
            self.sender_id = sender_id

main = svea_simulator.main

if __name__ == '__main__':
    main()
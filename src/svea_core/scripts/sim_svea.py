<<<<<<< HEAD
<<<<<<< HEAD
#!/usr/bin/env python3

=======
>>>>>>> 11eeed9 (Fundamental changes.)
=======
#!/usr/bin/env python3

>>>>>>> 4b0286b (More work on simulator)
"""
Simulation module for the SVEA platform. Creates fake ROS
subscriptions and publications that match the real car platform.
Intended for debugging code BEFORE running on a real car.

Author: Frank Jiang
"""

import math
import numpy as np
<<<<<<< HEAD
=======
from threading import Thread
>>>>>>> 11eeed9 (Fundamental changes.)

import rclpy
import rclpy.clock
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import tf2_ros
<<<<<<< HEAD
<<<<<<< HEAD
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int8

from svea_core import rosonic as rx
from svea_core.models.bicycle import Bicycle4DWithESC

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


qos_subber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep the last N messages
    durability=QoSDurabilityPolicy.VOLATILE,    # Volatile
    depth=10,                                   # Size of the queue
)

qos = QoSProfile(depth=10)

=======
=======
from tf_transformations import quaternion_from_euler
>>>>>>> 4b0286b (More work on simulator)
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from svea_core import rosonic as rx
from svea_core.states import SVEAControlValues
from svea_core.models.bicycle import Bicycle4DWithESC
from svea_msgs.msg import LLIControl as LLIControl
from svea_msgs.msg import LLIEmergency as LLIEmergency


qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


qos_subber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep the last N messages
    durability=QoSDurabilityPolicy.VOLATILE,    # Volatile
    depth=10,                                   # Size of the queue
)

<<<<<<< HEAD
>>>>>>> 11eeed9 (Fundamental changes.)
=======

>>>>>>> c56a41e (progressing on simulation, still not working)
class sim_svea(rx.Node):
    """
    Handles simulation of a SVEA vehicle. The object takes in a model
    and pretends to be the low-level of a SVEA vehicle. It will spin up
<<<<<<< HEAD
    a continuous loop that updates the state of the flake vehicle using
=======
    a continuous loop that updates the state of the fake vehicle using
>>>>>>> 11eeed9 (Fundamental changes.)
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

<<<<<<< HEAD
<<<<<<< HEAD
    time_step = 0.025
    publish_tf = rx.Parameter(True)

    steering_request_top = rx.Parameter('lli/ctrl/steering')
    throttle_request_top = rx.Parameter('lli/ctrl/throttle')
    highgear_request_top = rx.Parameter('lli/ctrl/highgear')
    diff_request_top = rx.Parameter('lli/ctrl/diff')

    odometry_top = rx.Parameter('odometry/local')

    map_frame = rx.Parameter('map')
    odom_frame = rx.Parameter('odom')
    self_frame = rx.Parameter('base_link')
    
    state = rx.Parameter([0.0, 0.0, 0.0, 0.0])  # x, y, yaw, velocity

    ## Publishers ##
    odometry_pub = rx.Publisher(Odometry, odometry_top, qos_pubber)

    ## Subscribers ##

    @rx.Subscriber(Int8, steering_request_top, qos_subber)
    def steering_request_cb(self, steering_request_msg):
        self.steering_req = steering_request_msg.data * -1
        self.last_ctrl_time = self.clock.now().to_msg()
    
    @rx.Subscriber(Int8, throttle_request_top, qos_subber)
    def throttle_request_cb(self, throttle_request_msg):
        self.velocity_req = throttle_request_msg.data
        self.last_ctrl_time = self.clock.now().to_msg()
    
    @rx.Subscriber(Bool, highgear_request_top, qos_subber)
    def highgear_request_cb(self, highgear_request_msg):
        self.highgear = highgear_request_msg.data
        self.last_ctrl_time = self.clock.now().to_msg()

    @rx.Subscriber(Bool, diff_request_top, qos_subber)
    def diff_request_cb(self, diff_request_msg):
        self.diff = diff_request_msg.data
        self.last_ctrl_time = self.clock.now().to_msg()

    ## Main Methods ##

    def on_startup(self):

        self.clock = rclpy.clock.Clock()
        self.last_ctrl_time = self.clock.now().to_msg()
        self.last_pub_time = self.clock.now().to_msg()

        self.model = Bicycle4DWithESC(initial_state=self.state)
        self.steering_req = 0.0
        self.velocity_req = 0.0
        self.highgear = False
        self.diff = False

        namespace = self.get_namespace()
        self.odom_frame = namespace + '/' + self.odom_frame if namespace != '/' else self.odom_frame
        self.self_frame = namespace + '/' + self.self_frame if namespace != '/' else self.self_frame

        if self.publish_tf:
            # for broadcasting fake tf tree
            self.tf_br = tf2_ros.TransformBroadcaster(self)
        
=======
    time_step = rx.Parameter(0.02, 'Time step for simulation')
    publish_tf = rx.Parameter(True, 'Publish TF tree')
=======
    time_step = rx.Parameter(0.02)
    publish_tf = rx.Parameter(True)
>>>>>>> 4b0286b (More work on simulator)

    state_top = rx.Parameter('state')
    request_top = rx.Parameter('lli/ctrl_request')
    actuated_top = rx.Parameter('lli/ctrl_actuated')
    emergency_top = rx.Parameter('lli/emergency')
    odometry_top = rx.Parameter('odometry/local')

    map_frame = rx.Parameter('map')
    odom_frame = rx.Parameter('odom')
    self_frame = rx.Parameter('base_link')

    ## Publishers ##

    ctrl_actuated_pub = rx.Publisher(LLIControl, actuated_top, qos_pubber)
    odometry_pub = rx.Publisher(Odometry, odometry_top, qos_pubber)

    ## Subscribers ##

    @rx.Subscriber(LLIControl, request_top, qos_subber)
    def ctrl_request_cb(self, ctrl_request_msg):
        print(self)
        print(ctrl_request_msg)
        
        self.last_ctrl_time = self.clock.now().to_msg()
        changed = self.inputs.update_from_msg(ctrl_request_msg)
        if changed:
            self.ctrl_actuated_pub.publish(self.inputs.ctrl_msg)

    @rx.Subscriber(LLIEmergency, emergency_top, qos_subber)
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

    ## Main Methods ##

    def on_startup(self):

        self.clock = rclpy.clock.Clock()
        self.last_ctrl_time = self.clock.now().to_msg()
        self.last_pub_time = self.clock.now().to_msg()

        self.model = Bicycle4DWithESC()
        self.inputs = SVEAControlValues(0, 0, 0, False, False)

        if self.publish_tf:
            # for broadcasting fake tf tree
            self.tf_br = tf2_ros.TransformBroadcaster(self)

        self.is_emergency = False
        self.sender_id = None
        
<<<<<<< HEAD
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        self.ctrl_actuated_pub = self.create_publisher(LLIControl,
                                                       self.actuated_top,
                                                       qos_profile)

        self.odometry_pub = self.create_publisher(Odometry,
                                                  self.odometry_top,
                                                  qos_profile)

        ## Subscribers ##
        
        self.create_subscription(LLIControl,
                                 self.request_top,
                                 self._update_ctrl_request,
                                 10)
        self.create_subscription(LLIEmergency,
                                 self.emergency_top,
                                 self._update_emergency,
                                 10)

>>>>>>> 11eeed9 (Fundamental changes.)
=======
>>>>>>> c56a41e (progressing on simulation, still not working)
        ## Timers ##

        self.create_timer(self.time_step, self.sim_loop)

    def sim_loop(self):
        curr_time = self.clock.now().to_msg()

<<<<<<< HEAD
<<<<<<< HEAD
        steering = self._percent_to_steer(self.steering_req / self.PERC_TO_LLI_COEFF)
        steering += np.random.normal(0, self.STEER_NOISE_STD)
        
        velocity = self._percent_to_vel(self.velocity_req / self.PERC_TO_LLI_COEFF)
=======
        steering = self._percent_to_steer(self.control_values.steering / self.PERC_TO_LLI_COEFF)
        steering += np.random.normal(0, self.STEER_NOISE_STD)
        
        velocity = self._percent_to_vel(self.control_values.velocity / self.PERC_TO_LLI_COEFF)
>>>>>>> 11eeed9 (Fundamental changes.)
=======
        steering = self._percent_to_steer(self.inputs.steering / self.PERC_TO_LLI_COEFF)
        steering += np.random.normal(0, self.STEER_NOISE_STD)
        
        velocity = self._percent_to_vel(self.inputs.velocity / self.PERC_TO_LLI_COEFF)
>>>>>>> 4b0286b (More work on simulator)
        velocity += np.random.normal(0, self.SPEED_NOISE_STD)
        
        if not (curr_time.sec - self.last_ctrl_time.sec) < self.CTRL_WAIT_TIME:
            steering, velocity = 0.0, 0.0

<<<<<<< HEAD
        self.model.update(steering, velocity, dt=self.time_step)

        # update the state message
        x, y, yaw, vel = self.model.state
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        odom_msg = Odometry()
        odom_msg.header.stamp = curr_time
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.self_frame
        odom_msg.header.stamp = curr_time
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.twist.twist.linear.x = vel
        
        # publish fake localization data
        if self.publish_tf:
            self._broadcast_tf(odom_msg)

        self.odometry_pub.publish(odom_msg)
        self.last_pub_time = self.clock.now().to_msg()
            
=======
        if self.is_emergency:
            velocity = 0

        self.model.update(steering, velocity, dt=self.time_step)

        # update the state message
        x, y, yaw, vel = self.model.state
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        odom_msg = Odometry()
        odom_msg.header.stamp = curr_time
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.self_frame
        odom_msg.header.stamp = curr_time
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.twist.twist.linear.x = vel
        
        # publish fake localization data
        if (curr_time.sec - self.last_pub_time.sec) > 1.0/self.LOC_PUB_FREQ:
            if self.publish_tf:
                self._broadcast_tf(odom_msg)

            self.odometry_pub.publish(odom_msg)
            self.last_pub_time = self.clock.now().to_msg()
>>>>>>> 11eeed9 (Fundamental changes.)

    def _percent_to_steer(self, steering):
        """Convert radians to percent of max steering actuation"""
        steering = float(steering)
        steer_percent = steering/100*self.MAX_STEERING_ANGLE
        return steer_percent

    def _percent_to_vel(self, vel_percent):
        vel_percent = float(vel_percent)
<<<<<<< HEAD
<<<<<<< HEAD
        if self.highgear == False:
            velocity = vel_percent*self.MAX_SPEED_0 / 100
        elif self.highgear == True:
            velocity = vel_percent*self.MAX_SPEED_1 / 100
        return velocity

    def _broadcast_tf(self, odom_msg):
        """Broadcast the tf tree for the fake localization data"""

        # broadcast the tf tree
        # map -> odom -> base_link

        map2odom = TransformStamped()
        map2odom.header.stamp = odom_msg.header.stamp
        map2odom.header.frame_id = self.map_frame
        map2odom.child_frame_id = self.odom_frame
=======
        if self.control_values.gear == 0:
=======
        if self.inputs.gear == 0:
>>>>>>> 4b0286b (More work on simulator)
            velocity = vel_percent*self.MAX_SPEED_0 / 100
        elif self.inputs.gear == 1:
            velocity = vel_percent*self.MAX_SPEED_1 / 100
        return velocity

    def _broadcast_tf(self, odom_msg):
        """Broadcast the tf tree for the fake localization data"""

        # broadcast the tf tree
        # map -> odom -> base_link

        map2odom = TransformStamped()
<<<<<<< HEAD
        map2odom.header.stamp = self.model.state.header.stamp
        map2odom.header.frame_id = self._map_frame_id
        map2odom.child_frame_id = self._odom_frame_id
>>>>>>> 11eeed9 (Fundamental changes.)
=======
        map2odom.header.stamp = odom_msg.header.stamp
        map2odom.header.frame_id = self.map_frame
        map2odom.child_frame_id = self.odom_frame
>>>>>>> 4b0286b (More work on simulator)
        map2odom.transform.rotation.w = 1.0
        self.tf_br.sendTransform(map2odom)

        odom2base = TransformStamped()
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 4b0286b (More work on simulator)
        odom2base.header.stamp = odom_msg.header.stamp
        odom2base.header.frame_id = self.odom_frame
        odom2base.child_frame_id = self.self_frame
        odom2base.transform.translation.x = odom_msg.pose.pose.position.x
        odom2base.transform.translation.y = odom_msg.pose.pose.position.y
        odom2base.transform.translation.z = odom_msg.pose.pose.position.z
        odom2base.transform.rotation = odom_msg.pose.pose.orientation
<<<<<<< HEAD
        self.tf_br.sendTransform(odom2base)

if __name__ == '__main__':
    sim_svea.main()
=======
        odom2base.header.stamp = self.model.state.header.stamp
        odom2base.header.frame_id = self._odom_frame_id
        odom2base.child_frame_id = self._base_link_frame_id
        pose = self.model.pose_msg.pose.pose
        odom2base.transform.translation.x = pose.position.x
        odom2base.transform.translation.y = pose.position.y
        odom2base.transform.translation.z = pose.position.z
        odom2base.transform.rotation = pose.orientation
=======
>>>>>>> 4b0286b (More work on simulator)
        self.tf_br.sendTransform(odom2base)

    @property
    def emergency(self):
        """Check if the emergency flag is set. Emulating actuation
        interface emergency check.

        :return: `True` if emergency flag is set, `False` otherwise,
                 `None` if no information has been received.
        :rtype: bool
        """
        return self.is_emergency

main = sim_svea.main

if __name__ == '__main__':
    main()
>>>>>>> 11eeed9 (Fundamental changes.)

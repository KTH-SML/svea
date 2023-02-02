"""
Simulation module for the SVEA platform. Creates fake ROS
subscriptions and publications that match the real car platform.
Intended for debugging code BEFORE running on a real car.
"""

import math
import numpy as np
from threading import Thread
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from svea_msgs.msg import lli_ctrl, lli_emergency
from svea.states import SVEAControlValues
from .sim_lidar import SimLidar

__license__ = "MIT"
__maintainer__ = "Frank Jiang"
__email__ = "frankji@kth.se"
__status__ = "Development"


class SimSVEA(object):
    """
    Handles simulation of a SVEA vehicle. The object takes in a model
    and pretends to be the low-level of a SVEA vehicle. It will spin up
    a continuous loop that updates the state of the fake vehicle using
    the model dynamics at a fixed rate and takes control inputs by
    subscribing to the same low-level interface ROS topic the actual
    SVEA low-level system uses and even publishes actuated control to
    the same topic as the low-level system.

    :param initialized_model: Model to use as pretend SVEA vehicle. The
                              only requirements on the model is that it
                              has an 'update' method that takes a
                              steering angle and a velocity as inputs
    :type initialized_model: Model
    :param vehicle_name: Name of vehicle, etc. SVEA0, used for creating
                         ROS topic names, defaults to ''
    :type vehicle_name: str
    :param dt: Sampling time in [s]
    :type dt: float
    :param start_paused: Start simulation paused
    :type start_paused: bool, optional
    :param run_lidar: Run simulated lidar alongside simulation
    :type run_lidar: bool
    :param publish_pose: Publish the simulated vehicles pose if `True`
                         Default: `False`
    :type publish_pose: bool, optional
    :param publish_odometry: Publish the simulated vehicles odometry if `True`
                         Default: `False`
    :type publish_odometry: bool, optional
    """

    LOC_PUB_FREQ = 50 # Hz
    CTRL_WAIT_TIME = 0.2 # s
    MAX_SPEED_0 = 1.7 # [m/s]
    MAX_SPEED_1 = 3.6 # [m/s]
    MAX_STEERING_ANGLE = 40*math.pi/180

    SPEED_NOISE_STD = 0.05
    STEER_NOISE_STD = 0.1

    # scaling factor, percentage to lli actuation
    PERC_TO_LLI_COEFF = 1.27

    def __init__(self,
                 initialized_model,
                 vehicle_name='',
                 dt=0.02,
                 start_paused=False,
                 run_lidar=False,
                 publish_tf = True,
                 publish_pose=False,
                 publish_odometry=False):

        sub_namespace = vehicle_name + '/' if vehicle_name else ''
        self._state_topic = sub_namespace + 'state'
        self._request_topic = sub_namespace + 'lli/ctrl_request'
        self._actuated_topic = sub_namespace + 'lli/ctrl_actuated'
        self._emergency_topic = sub_namespace + 'lli/emergency'

        if vehicle_name:
            self.vehicle_name = vehicle_name
        else:
            namespace = rospy.get_namespace()
            self.vehicle_name = namespace.split('/')[-2]

        self._map_frame_id = 'map'
        self._odom_frame_id = sub_namespace + 'odom'
        self._base_link_frame_id = sub_namespace + 'base_link'

        self.model = initialized_model
        self.dt = dt
        self.current_state = self.model.state
        self.current_state.frame_id = self._map_frame_id
        self.current_state.child_frame_id = self._base_link_frame_id

        self.publish_tf = publish_tf
        if self.publish_tf:
            # for broadcasting fake tf tree
            self.tf_br = tf2_ros.TransformBroadcaster()
        self.publish_pose = publish_pose
        if self.publish_pose:
            self._pose_topic = sub_namespace + 'pose'
        self.publish_odometry = publish_odometry
        if self.publish_odometry:
            self._odometry_topic = sub_namespace + 'odometry/corrected'
        self._last_pub_time = rospy.get_time()

        self.node_name = "simulated_" + self.vehicle_name
        self.is_pause = start_paused
        self.control_values = SVEAControlValues(0, 0, 0, False, False)

        self._last_ctrl_time = rospy.get_time()

        self.is_emergency = False
        self.sender_id = None

        self._run_lidar = run_lidar
        if self._run_lidar:
            self.simulated_lidar = SimLidar(vehicle_name).start()

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data

        :return: itself
        :rtype: SimSVEA
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Simulation Node: \n"
                      + str(self))
        self._collect_srvs()
        self._start_publish()
        rospy.loginfo("{} Simulation successfully initialized".format(
            self.vehicle_name))
        self._start_listen()
        rospy.spin()

    def _collect_srvs(self):
        pass

    def _start_listen(self):
        rospy.Subscriber(self._request_topic,
                         lli_ctrl,
                         self._update_ctrl_request)
        rospy.Subscriber(self._emergency_topic,
                         lli_emergency,
                         self._update_emergency)
        self._start_simulation()

    def _start_publish(self):
        self.svea_state_pub = rospy.Publisher(self._state_topic,
                                              type(self.current_state.state_msg),
                                              queue_size=1,
                                              tcp_nodelay=True)
        self.ctrl_actuated_pub = rospy.Publisher(self._actuated_topic,
                                                 lli_ctrl,
                                                 queue_size=1,
                                                 tcp_nodelay=True)
        if self.publish_pose:
            self.pose_pub = rospy.Publisher(self._pose_topic,
                                            type(self.current_state.pose_msg),
                                            queue_size=1,
                                            tcp_nodelay=True)
        if self.publish_odometry:
            self.odometry_pub = rospy.Publisher(self._odometry_topic,
                                                type(self.current_state.odometry_msg),
                                                queue_size=1,
                                                tcp_nodelay=True)

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

    def toggle_pause_simulation(self):
        """Toggle between pause and play simulation"""
        self.is_pause = not self.is_pause
        status = "paused" if self.is_pause else "playing"
        rospy.loginfo("Simulation is now {0}".format(status))

    def _start_simulation(self):
        rate = rospy.Rate(1/self.dt)
        while not rospy.is_shutdown():
            curr_time = rospy.get_time()
            if not self.is_pause:
                steer_percent = self.control_values.steering / self.PERC_TO_LLI_COEFF
                vel_percent = self.control_values.velocity / self.PERC_TO_LLI_COEFF
                steering = self._percent_to_steer(steer_percent) \
                    + np.random.normal(0, self.STEER_NOISE_STD)
                velocity = self._percent_to_vel(vel_percent) \
                    + np.random.normal(0, self.SPEED_NOISE_STD)
                if not self.is_emergency:
                    if not (curr_time - self._last_ctrl_time) < self.CTRL_WAIT_TIME:
                        steering, velocity = 0, 0
                    self.model.update(steering, velocity, self.dt)
                else:
                    self.model.update(steering, 0, self.dt)

                # update lidar position
                if self._run_lidar:
                    self.simulated_lidar.update_lidar_position(self.model.state)

                # publish fake localization data
                if (curr_time - self._last_pub_time) > 1.0/self.LOC_PUB_FREQ:
                    self.svea_state_pub.publish(self.current_state.state_msg)
                    if self.publish_tf:
                        self._broadcast_tf()
                    if self.publish_pose:
                        self.pose_pub.publish(self.current_state.pose_msg)
                    if self.publish_odometry:
                        self.odometry_pub.publish(self.current_state.odometry_msg)
                    self._last_pub_time = rospy.get_time()
            rate.sleep()  # force update frequency to be realistic

    def _broadcast_tf(self):
        map2odom = TransformStamped()
        map2odom.header.stamp = self.current_state.pose_msg.header.stamp
        map2odom.header.frame_id = self._map_frame_id
        map2odom.child_frame_id = self._odom_frame_id
        map2odom.transform.rotation.w = 1.0
        self.tf_br.sendTransform(map2odom)

        odom2base = TransformStamped()
        odom2base.header.stamp = self.current_state.pose_msg.header.stamp
        odom2base.header.frame_id = self._odom_frame_id
        odom2base.child_frame_id = self._base_link_frame_id
        pose = self.current_state.pose_msg.pose.pose
        odom2base.transform.translation.x = pose.position.x
        odom2base.transform.translation.y = pose.position.y
        odom2base.transform.translation.z = pose.position.z
        odom2base.transform.rotation = pose.orientation
        self.tf_br.sendTransform(odom2base)


    def _update_ctrl_request(self, ctrl_request_msg):
        self._last_ctrl_time = rospy.get_time()
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

    def _build_param_printout(self):
        param_str = "### {0} Simulation:\n".format(self.vehicle_name)
        param_str += "### Sim dt: {0}\n".format(self.dt)
        param_str += "  -vehicle state:\n"
        param_str += "{0}\n".format(self.model)
        param_str += "  -ctrl request:\n"
        param_str += str(self.control_values)
        return param_str

    def __repr__(self):
        return self._build_param_printout()

    def __str__(self):
        return self._build_param_printout()

#!/usr/bin/env python3

"""
Simulation module for the SVEA platform. Creates fake ROS
subscriptions and publications that match the real car platform.
Intended for debugging code BEFORE running on a real car.

Contributors:
- Frank Jiang
- Kaj Munhoz Arfvidsson
- Li Chen
"""

import math
import numpy as np

import rclpy
import rclpy.clock
import tf2_ros
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import ManualControl

from svea_core import rosonic as rx
from svea_core.models.bicycle import Bicycle4DWithESC

class sim_svea(rx.Node):
    """
    Handles simulation of a SVEA vehicle. The object takes in a model
    and pretends to be the low-level of a SVEA vehicle. It will spin up
    a continuous loop that updates the state of the flake vehicle using
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

    time_step = 0.025

    initial_pose_x = rx.Parameter(0.0)
    initial_pose_y = rx.Parameter(0.0)
    initial_pose_a = rx.Parameter(0.0)

    odometry_top = rx.Parameter('odometry/local')
    control_top = rx.Parameter('mavros/manual_control/send')

    map_frame = rx.Parameter('map')
    odom_frame = rx.Parameter('odom')
    base_frame = rx.Parameter('base_link')

    publish_tf = rx.Parameter(True)

    ## Publishers ##
    odometry_pub = rx.Publisher(Odometry, odometry_top)

    ## Subscribers ##

    @rx.Subscriber(ManualControl, control_top)
    def recv_control(self, msg: ManualControl):
        """Receive control inputs from the low-level interface."""
        self.steering_req = - msg.y / 10.0
        self.velocity_req = (500 - msg.z) / 5.0
        self.highgear = msg.aux3 < 0
        self.diff = msg.aux1 > 0 and msg.aux2 < 0
        self.last_ctrl_time = self.clock.now().to_msg()

    ## Main Methods ##

    def on_startup(self):

        self.clock = rclpy.clock.Clock()
        self.last_ctrl_time = self.clock.now().to_msg()
        self.last_pub_time = self.clock.now().to_msg()

        self.model = Bicycle4DWithESC(initial_state=[
            self.initial_pose_x,
            self.initial_pose_y,
            self.initial_pose_a,
            0.0,
        ])
        self.steering_req = 0.0
        self.velocity_req = 0.0
        self.highgear = False
        self.diff = False

        if self.publish_tf:
            # for broadcasting fake tf tree
            self.tf_br = tf2_ros.TransformBroadcaster(self)
        
        ## Timers ##

        self.create_timer(self.time_step, self.sim_loop)

    def sim_loop(self):
        curr_time = self.clock.now().to_msg()

        steering = self._percent_to_steer(self.steering_req / self.PERC_TO_LLI_COEFF)
        steering += np.random.normal(0, self.STEER_NOISE_STD)
        
        velocity = self._percent_to_vel(self.velocity_req / self.PERC_TO_LLI_COEFF)
        velocity += np.random.normal(0, self.SPEED_NOISE_STD)
        
        if not (curr_time.sec - self.last_ctrl_time.sec) < self.CTRL_WAIT_TIME:
            steering, velocity = 0.0, 0.0

        self.model.update(steering, velocity, dt=self.time_step)

        # update the state message
        x, y, yaw, vel = self.model.state
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        odom_msg = Odometry()
        odom_msg.header.stamp = curr_time
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.base_frame
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
            

    def _percent_to_steer(self, steering):
        """Convert radians to percent of max steering actuation"""
        steering = float(steering)
        steer_percent = steering/100*self.MAX_STEERING_ANGLE
        return steer_percent

    def _percent_to_vel(self, vel_percent):
        vel_percent = float(vel_percent)
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
        map2odom.transform.rotation.w = 1.0
        self.tf_br.sendTransform(map2odom)

        odom2base = TransformStamped()
        odom2base.header.stamp = odom_msg.header.stamp
        odom2base.header.frame_id = self.odom_frame
        odom2base.child_frame_id = self.base_frame
        odom2base.transform.translation.x = odom_msg.pose.pose.position.x
        odom2base.transform.translation.y = odom_msg.pose.pose.position.y
        odom2base.transform.translation.z = odom_msg.pose.pose.position.z
        odom2base.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_br.sendTransform(odom2base)

if __name__ == '__main__':
    sim_svea.main()

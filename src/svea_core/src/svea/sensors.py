#!/usr/bin/env python

"""
Sensor module that contains ROS interface objects for various sensors.
Currently supporting: Lidars, Wheel Encoders
"""

from threading import Thread
import math
import rospy
from sensor_msgs.msg import LaserScan
from svea_msgs.msg import lli_encoder
from geometry_msgs.msg import TwistWithCovarianceStamped


__author__ = "Frank Jiang and Tobias Bolin"
__copyright__ = "Copyright 2020, Frank Jiang"
__credits__ = ["Frank Jiang", "Tobias Bolin"]
__license__ = "MIT"
__maintainer__ = "Frank Jiang"
__email__ = "frankji@kth.se"
__status__ = "Development"


class Lidar():
    """
    Basic interface for handling a Lidar. Collects and stores the most recent
    scan.
    """

    def __init__(self):
        self.scan = []
        # list of functions to call whenever a new scan comes in
        self.callbacks = []

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data

        :return: itself
        :rtype: Lidar
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Lidar Interface Node: \n" + str(self))
        self._collect_srvs()
        self._start_listen()

    def _collect_srvs(self):
        pass

    def _start_listen(self):
        rospy.Subscriber('scan', LaserScan, self._read_scan,
                         tcp_nodelay=True)
        rospy.loginfo("Lidar Interface successfully initialized")
        rospy.spin()

    def _read_scan(self, scan_msg):
        self.scan = scan_msg.ranges

        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment

        self.time_increment = scan_msg.time_increment

        self.last_scan_time = scan_msg.scan_time

        for cb in self.callbacks:
            cb(self.scan, self.angle_min, self.angle_increment)

    def add_callback(self, cb):
        """Add state callback. Every function passed into this method
        will be called whenever new scan information comes in from the
        Lidar driver.

        :param cb: A callback function intended for responding to the
                   reception of a new scan, function must accept list
                   of scans, min angle, and angle increment as arguments
        :type cb: function
        """
        self.callbacks.append(cb)

    def remove_callback(self, cb):
        """Remove callback so it will no longer be called when state
        information is received

        :param cb: A callback function that should be no longer used
                   in response to the reception of state info
        :type cb: function
        """
        while cb in self.callbacks:
            self.callbacks.pop(self.callbacks.index(cb))


class WheelEncoder():
    """Interface for wheel encoders

    :param vehicle_name: name of the vehicle, defaults to ''
    :type vehicle_name: str, optional
    :param encoder_frame: Transform frame id of the encoders,
        defaults to 'base_link'
    :type encoder_frame: str, optional
    :param encoder_topic: Topic that encoder messages should be read from,
        defaults to 'lli/encoder'
    :type encoder_topic: str, optional
    :param direction_topic: Topic with twist messages
        used for calculating the direction, defaults to ''
    :type direction_topic: str, optional
    :param axle_track: Whidth between the wheels in mm,
        defaults to 199.0
    :type axle_track: float, optional
    :param wheel_radius: Radius of the wheels in mm,
        defaults to 60.0
    :type wheel_radius: float, optional
    :param ticks_per_revolution: Number of encoder ticks in one revolution of a wheel,
        defaults to 60
    :type ticks_per_revolution: int, optional
    :param linear_covariance: Covariance of the linear velocity in the published twist messages,
        defaults to 0.2
    :type linear_covariance: float, optional
    :param angular_covariance: Covariance of the angular velocity
        in the published twist messages, defaults to 0.4
    :type angular_covariance: float, optional
    """

    def __init__(self,
                 vehicle_name='',
                 encoder_frame='base_link',
                 encoder_topic='lli/encoder',
                 direction_topic='',
                 axle_track=199.0,
                 wheel_radius=60.0,
                 ticks_per_revolution=60,
                 linear_covariance=0.2,
                 angular_covariance=0.4,
                 ):
        if vehicle_name:
            self.vehicle_name = vehicle_name
        else:
            namespace = rospy.get_namespace()
            self.vehicle_name = namespace.split('/')[-1]
        self.linear_covariance = linear_covariance
        self.angular_covariance = angular_covariance
        self.encoder_topic = encoder_topic
        self.direction_topic = direction_topic
        self.frame_id = encoder_frame
        # Vehicle parameters
        mm_to_meter = 1e-3
        tau = 2 * math.pi
        self.axle_track = axle_track * mm_to_meter
        self.wheel_radius = wheel_radius
        self.ticks_per_revolution = ticks_per_revolution
        self.tick_to_distance_coefficient = (
            wheel_radius * tau * mm_to_meter / ticks_per_revolution
        )
        # Storage fields
        self.direction = 1
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.r_wheel_velocity = 0.0
        self.l_wheel_velocity = 0.0
        # list of functions to call whenever a new reading comes in
        self.callbacks = []

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data

        :return: itself
        :rtype: WheelEncoderInterface
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo('Starting wheel encoder Interface for'
                      + self.vehicle_name)
        self._collect_srvs()
        self._start_listen()
        rospy.loginfo('Succesfully initiated wheel encoder Interface for'
                      + self.vehicle_name)

    def _collect_srvs(self):
        pass

    def _start_listen(self):
        self.encoder_subscriber = rospy.Subscriber(
            self.encoder_topic,
            lli_encoder,
            self._process_encoder_data,
            tcp_nodelay=True)
        if self.direction_topic:
            self.actuation_subscriber = rospy.Subscriber(
                self.direction_topic,
                TwistWithCovarianceStamped,
                self._process_direction,
                tcp_nodelay=True)

    def _process_encoder_data(self, msg):
        right_wheel_velocity = self._calc_wheel_velocity(
            msg.right_ticks,
            msg.right_time_delta)
        left_wheel_velocity = self._calc_wheel_velocity(
            msg.left_ticks,
            msg.left_time_delta)
        direction = self.direction
        # Linear velocity
        self.linear_velocity = (right_wheel_velocity + left_wheel_velocity)/2
        self.linear_velocity *= direction
        # Angular velocity
        angular_velocity = (right_wheel_velocity - left_wheel_velocity)
        angular_velocity /= self.axle_track
        angular_velocity *= direction
        self.angular_velocity = angular_velocity
        for cb in self.callbacks:
            cb(self)

    def _process_direction(self, msg):
        velocity = msg.twist.twist.linear.x
        direction_epsilon = self.tick_to_distance_coefficient * 0.5 # m/s
        if velocity > direction_epsilon:
            self.direction = 1
        elif velocity < direction_epsilon:
            self.direction = -1
        else:
            self.directions = 0

    def _calc_wheel_velocity(self, ticks, time_delta):
        if time_delta == 0:
            return 0
        distance = ticks * self.tick_to_distance_coefficient
        velocity = (distance/time_delta) * 1e6
        return velocity

    def add_callback(self, cb):
        """Add a callback. Every function passed into this method
        will be called whenever new information comes in from the sensor.

        :param cb: A callback function intended for responding to the
                   reception of a new reading.

                   The function should take a WheelEncoder object
                   as input.
        :type cb: function
        :return: Handle to the callback function
        """
        self.callbacks.append(cb)

    def remove_callback(self, cb):
        """Remove callback so it will no longer be called when state
        information is received

        :param cb: A callback function that should be no longer used
                   in response to the reception of state info
        :type cb: function
        """
        while cb in self.callbacks:
            self.callbacks.pop(self.callbacks.index(cb))
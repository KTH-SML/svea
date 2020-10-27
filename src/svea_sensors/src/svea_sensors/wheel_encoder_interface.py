#!/usr/bin/env python
import math
import rospy

from svea_arduino.msg import lli_encoder
from geometry_msgs.msg import TwistWithCovarianceStamped

# Wheel base in mm
DEFAULT_WHEEL_BASE = 340.0
# Wheel radius in mm
DEFAULT_WHEEL_RADIUS = 60.0
TICKS_PER_REVOLUTION = 60

DEFAULT_LINEAR_COVARIANCE = 0.1
DEFAULT_ANGULAR_COVARIANCE = 10.0

class WheelEncoderInterface(object):
    def __init__(self, 
                 wheel_base=DEFAULT_WHEEL_BASE,
                 wheel_radius=DEFAULT_WHEEL_RADIUS,
                 vehicle_name='',
                 linear_covariance = DEFAULT_LINEAR_COVARIANCE,
                 angular_covariance = DEFAULT_ANGULAR_COVARIANCE):
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        if vehicle_name:
            self.vehicle_name = vehicle_name
        else:
            namespace = rospy.get_namespace()
            self.vehicle_name = namespace.split('/')[-1]
        self.linear_covariance = linear_covariance
        self.angular_covariance = angular_covariance
        self.encoder_topic = 'lli/encoder'
        self.twist_topic = 'wheel_encoder_twist'
        self.node_name = 'wheel_encoder_interface'
        self.tick_to_distance_coefficient = (
            DEFAULT_WHEEL_RADIUS * 2 * math.pi / TICKS_PER_REVOLUTION
        )
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def start(self, block=False):
        rospy.loginfo('Starting wheel encoder Interface Node for'
                      + self.vehicle_name)
        self._start_publish()
        self._start_listen()
        rospy.sleep(0.1)
        # self._wait_until_ready()
        # if not self.is_ready:
        #     rospy.logwarn("LLI interface not responding during start of "
        #                   "Control Interface. Seting ready anyway.")
        # self.is_ready = True
        # rospy.loginfo("{} Control Interface successfully initialized"
        #               .format(self.vehicle_name))
        if block:
            rospy.spin()

    # def _init_and_spin_ros(self):
    #     rospy.loginfo('Starting wheel encoder Interface Node for' 
    #                    + self.vehicle_name)
    #     self._start_publish()
    #     self._start_listen()
    #     rospy.sleep(0.1)
    #     # self._wait_until_ready()
    #     # if not self.is_ready:
    #     #     rospy.logwarn("LLI interface not responding during start of "
    #     #                   "Control Interface. Seting ready anyway.")
    #     # self.is_ready = True
    #     # rospy.loginfo("{} Control Interface successfully initialized"
    #     #               .format(self.vehicle_name))
    #     rospy.spin()

    def _start_listen(self):
        self.encoder_subscriber = rospy.Subscriber(
            self.encoder_topic,
            lli_encoder,
            self._process_encoder_data,
            tcp_nodelay=True)

    def _start_publish(self):
        self.twist_publisher = rospy.Publisher(
            self.twist_topic,
            TwistWithCovarianceStamped,
            queue_size=1,
            tcp_nodelay=True)

    def _calc_wheel_velocity(self, ticks, time_delta):
        if time_delta == 0: 
          return 0
        distance = ticks * self.tick_to_distance_coefficient
        return (distance/time_delta) * 1e6

    def _set_covariance(self, covariance):
        for i in range(0, 15, 7):
            covariance[i] = self.linear_covariance
        for i in range(14+7, 15+7*3, 7):
            covariance[i] = self.angular_covariance

    def _process_encoder_data(self, msg):
        twist_msg = TwistWithCovarianceStamped()
        right_wheel_velocity = self._calc_wheel_velocity(msg.right_ticks,
                                                         msg.right_time_delta)
        left_wheel_velocity = self._calc_wheel_velocity(msg.left_ticks,
                                                        msg.left_time_delta)
        self.linear_velocity = (right_wheel_velocity + left_wheel_velocity)/2
        #TODO calculate angular velocity here
        self.angular_velocity = 0.0
        self._set_covariance(twist_msg.twist.covariance)
        #twist_msg.header = msg.header
        twist_msg.twist.twist.linear.x = self.linear_velocity
        twist_msg.twist.twist.angular.z = self.angular_velocity
        self.twist_publisher.publish(twist_msg)

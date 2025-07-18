"""
Author: Frank Jiang, Tobias Bolin
"""
from threading import Event
from collections import deque
from math import pi, isnan
from typing import Optional, Self

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.node import Node

from enum import IntEnum
from .. import rosonic as rx
from std_msgs.msg import Bool, Int8

QoS_DEFAULT = QoSProfile(depth = 10)
QoS_RELIABLE = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
)

class Controls(IntEnum):
    """Enum for control codes sent to the low-level interface."""
    STEERING = 0
    VELOCITY = 1


class ActuationInterface(rx.Resource):

    # saturation input limits to keep hardware healthy
    MAX_STEER_PERCENT = 90          # [%]
    MAX_SPEED_PERCENT = 90          # [%]
    # assumed maximum steering angle, approximately 40 degrees
    MAX_STEERING_ANGLE = 40*pi/180  # [rad]
    # By testing, the max velocity in Gear 0 is around 1.7 m/s.
    # The max velocity in Gear 2 is around 3.6 m/s.
    MAX_SPEED_0 = 1.7               # [m/s]
    MAX_SPEED_1 = 3.6               # [m/s]

    steering_pub = rx.Publisher(Int8, 'lli/ctrl/steering', qos_profile=QoS_DEFAULT)
    throttle_pub = rx.Publisher(Int8, 'lli/ctrl/throttle', qos_profile=QoS_DEFAULT)
    highgear_pub = rx.Publisher(Bool, 'lli/ctrl/highgear', qos_profile=QoS_RELIABLE)
    diff_pub = rx.Publisher(Bool, 'lli/ctrl/diff', qos_profile=QoS_RELIABLE)

    def __init__(self, rate=20, use_acceleration=False, highgear=False, diff=False):
        self.acceleration = use_acceleration
        self.rate = rate
        self.latest_controls = [0., 0.] # [steering, velocity]
        self.highgear_msg = Bool()
        self.highgear_msg.data = highgear
        self.diff_msg = Bool()
        self.diff_msg.data = diff
        self.steering_msg = Int8()
        self.steering_msg.data = 0
        self.velocity_msg = Int8()
        self.velocity_msg.data = 0

    def on_startup(self):
        self.node.get_logger().info("Starting Actuation Interface Node...")
        self.node.create_timer(1/self.rate, self.loop)
        self.highgear_pub.publish(self.highgear_msg)
        self.diff_pub.publish(self.diff_msg)
        self.node.get_logger().info("Actuation Interface is ready.")

    def loop(self):
        """Main loop to publish control messages."""
        self.steering_msg.data = int(self.latest_controls[Controls.STEERING] * 1.27)  # 1.27 is a factor to convert from percent to the range of -127 to 127
        self.velocity_msg.data = int(self.latest_controls[Controls.VELOCITY] * 1.27) # 1.27 is a factor to convert from percent to the range of -127 to 127
        self.steering_pub.publish(self.steering_msg)
        self.throttle_pub.publish(self.velocity_msg)

    def send_control(self,
                     steering:Optional[float] = None,
                     vel_or_acc:Optional[float] = None):
        """Send control inputs to the low-level interface."""
        # Steering
        if steering is not None and not isnan(steering):
            steer_percent = self._steer_to_percent_and_clip(steering) 
            self.__send_steering(steer_percent)
        
        if vel_or_acc is not None or not isnan(vel_or_acc):
            if self.acceleration:
                self.__send_acceleration(vel_or_acc)                            
            else:
                vel_percent = self._speed_to_percent(vel_or_acc) 
                vel_percent = self._speed_clip(vel_percent)
                self.__send_velocity(vel_percent)
            
            
            
    def __send_steering(self, steering):
        self.latest_controls[Controls.STEERING] = steering

    def __send_velocity(self, velocity):
        self.latest_controls[Controls.VELOCITY] = velocity

    def __send_acceleration(self, acceleration):        
        acc_percent = self._speed_to_percent(acceleration) 
        vel_percent = self._speed_clip(self.latest_controls[Controls.VELOCITY] + acc_percent)
        self.__send_velocity(vel_percent)


    def toggle_highgear(self):
        """Toggle the high gear state."""
        if self.highgear_msg.data:
            self.disable_highgear()
        else:
            self.enable_highgear()

    def enable_highgear(self):
        """Enable high gear."""
        self.highgear_msg.data = True
        self.highgear_pub.publish(self.highgear_msg)

    def disable_highgear(self):
        """Disable high gear."""
        self.highgear_msg.data = False
        self.highgear_pub.publish(self.highgear_msg)

    def toggle_diff(self):
        """Toggle the differential lock state."""
        if self.diff_msg.data:
            self.disable_diff()
        else:
            self.enable_diff()

    def enable_diff(self): 
        """Enable the differential lock."""
        self.diff_msg.data = True
        self.diff_pub.publish(self.diff_msg)

    def disable_diff(self):
        """Disable the differential lock."""
        self.diff_msg.data = False
        self.diff_pub.publish(self.diff_msg)

    def _steer_to_percent_and_clip(self, steering):
        """Convert radians to percent of max steering actuation"""
        steering = float(steering)
        steer_percent = steering*100.0/self.MAX_STEERING_ANGLE
        return min(self.MAX_STEER_PERCENT,
                               max(-self.MAX_STEER_PERCENT, steer_percent))
    
    def _speed_to_percent(self, speed):
        """Convert m/s to percent of max speed actuation"""
        speed = float(speed)
        speed_percent = speed*100.0/self.max_speed
        return speed_percent
    
    def _speed_clip(self, speed):
        """Clip speed to the maximum speed"""
        return min(self.MAX_SPEED_PERCENT,
                               max(-self.MAX_SPEED_PERCENT, speed))

    @property
    def max_speed(self) -> float:
        """Get the maximum speed, dependent on gear.

        Returns:
            The maximum speed, independent of direction
        """
        return self.MAX_SPEED_1 if self.highgear_msg.data else self.MAX_SPEED_0
"""
Author: Frank Jiang, Tobias Bolin
"""
from math import pi, isnan
from typing import Optional

from rclpy.qos import QoSProfile
from enum import IntEnum
from .. import rosonic as rx
from mavros_msgs.msg import ManualControl


class Controls(IntEnum):
    """Enum for control codes sent to the low-level interface."""
    STEERING = 0
    VELOCITY = 1


class ActuationInterface(rx.Field):
    r"""Interface object for sending actuation commands to the SVEA car's low-level
    controller.

    We implement the interface for two reasons:

    1. Our models typically expect steering angle `[rad]` and velocity `[m/s]`.
    2. We would like to add some features on top of just sending the control
       inputs.

    The low-level controller expects unit-less steering and velocity values between
    `[-127, 127]` that correspond to the minimum and maximum steering angles and
    velocities.

    This interface makes it easy to input steering angles in `[rad]` and
    velocities in `[m/s]`. Assumes very simplistic models of the low-level
    actuation. It assumes a linear steering model and a linear velocity model.
    The control interface assumes that the ESC is in the default Sports mode.
    """
    # saturation input limits to keep hardware healthy
    MAX_STEER_PERCENT = 90          # [%]
    MAX_SPEED_PERCENT = 90          # [%]
    # assumed maximum steering angle, approximately 40 degrees
    MAX_STEERING_ANGLE = 40*pi/180  # [rad]
    # By testing, the max velocity in Gear 0 is around 1.7 m/s.
    # The max velocity in Gear 2 is around 3.6 m/s.
    MAX_SPEED_0 = 1.7               # [m/s]
    MAX_SPEED_1 = 3.6               # [m/s]

    actuation = rx.namespace(
        control_top = rx.Parameter('mavros/manual_control/send'),
    )

    control_pub = rx.Publisher(ManualControl, actuation.control_top)

    def __init__(self, rate=20, use_acceleration=False, highgear=False, difflock=False):
        assert 2 <= rate <= 25, 'Actuation Interface: Publish rate outside of admissible bounds.'
        self.rate = rate
        self.acceleration = use_acceleration
        self.steering_percent = 0.0
        self.velocity_percent = 0.0
        self.highgear = highgear
        self.difflock = difflock

    def on_startup(self):
        self.node.get_logger().info("Starting Actuation Interface Node...")
        self.node.create_timer(1/self.rate, self.loop)
        self.node.get_logger().info("Actuation Interface is ready.")

    def loop(self):
        """Main loop to publish ManualControl messages."""
        msg = ManualControl()
        
        # Map steering percent to y channel [-1000, 1000]
        msg.y = - float(self.steering_percent * 10)
        
        # Map velocity percent to z channel [0, 1000], 500 is neutral
        msg.z = 1000 - float(500 + self.velocity_percent * 5)
        
        # Set enabled_extensions for SVEA (Enables aux1 and aux2 channels)
        msg.enabled_extensions = 252
        
        # Differential lock: aux1 (front) and aux2 (rear, inverted)
        if self.difflock:
            msg.aux1 = 1000.   # front diff ON
            msg.aux2 = -1000.  # rear diff ON (inverted)
        else:
            msg.aux1 = -1000.
            msg.aux2 = 1000.
        
        # High gear: aux3
        if self.highgear:
            msg.aux3 = -1000.
        else:
            msg.aux3 = 1000.

        self.control_pub.publish(msg)

    def send_control(self,
                     steering:Optional[float] = None,
                     vel_or_acc:Optional[float] = None):
        """Send control inputs to the low-level interface."""
        # Steering
        if steering is not None and not isnan(steering):
            steer_percent = self._steer_to_percent_and_clip(steering) 
            self.__send_steering(steer_percent)
        
        if vel_or_acc is not None and not isnan(vel_or_acc):
            if self.acceleration:
                self.__send_acceleration(vel_or_acc)                            
            else:
                vel_percent = self._speed_to_percent(vel_or_acc) 
                vel_percent = self._speed_clip(vel_percent)
                self.__send_velocity(vel_percent)
    
    def __send_steering(self, steering):
        self.steering_percent = steering

    def __send_velocity(self, velocity):
        self.velocity_percent = velocity

    def __send_acceleration(self, acceleration):        
        acc_percent = self._speed_to_percent(acceleration) 
        vel_percent = self._speed_clip(self.velocity_percent + acc_percent)
        self.__send_velocity(vel_percent)


    def toggle_highgear(self):
        """Toggle the high gear state."""
        self.highgear = not self.highgear

    def enable_highgear(self):
        """Enable high gear."""
        self.highgear = True

    def disable_highgear(self):
        """Disable high gear."""
        self.highgear = False

    def toggle_difflock(self):
        """Toggle the differential lock state."""
        self.difflock = not self.difflock

    def enable_difflock(self): 
        """Enable the differential lock."""
        self.difflock = True

    def disable_difflock(self):
        """Disable the differential lock."""
        self.difflock = False

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
        return self.MAX_SPEED_1 if self.highgear else self.MAX_SPEED_0

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 54289ac (2025/04/16 Meeting Update)
"""
Author: Frank Jiang, Tobias Bolin
"""
from threading import Event
<<<<<<< HEAD
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


class ActuationInterface(rx.Field):
    r"""Interface object for sending actuation commands to the SVEA car's low-level
=======
from threading import Thread, Event
=======
>>>>>>> 54289ac (2025/04/16 Meeting Update)
from collections import deque
from math import pi, isnan
from typing import Optional, Self

import rclpy
import rclpy.clock
import rclpy.duration
from rclpy.node import Node
from svea_msgs.msg import LLIControl
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

__all__ = [
    'ActuationInterface',
]


def cmp(a, b):
    return (a > b) - (a < b)


class ActuationInterface:
    """Interface object for sending actuation commands to the SVEA car's low-level
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)
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
<<<<<<< HEAD
    """
=======

    Args:
        vehicle_name: Name of vehicle being controlled; The name will be
            effectively be added as a namespace to the topics used by the
            corresponding low lever interface i.e
            `namespace/vehicle_name/lli/topic`.
        log_length: Number of messages from control requests, control actuated
            and the remote that should be stored in the internal log. Set to
            `None` for unlimited logging.
    """

    # arduino's expected input frequency
    OPERATING_FREQ = 50             # [Hz]
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)
    # saturation input limits to keep hardware healthy
    MAX_STEER_PERCENT = 90          # [%]
    MAX_SPEED_PERCENT = 90          # [%]
    # assumed maximum steering angle, approximately 40 degrees
    MAX_STEERING_ANGLE = 40*pi/180  # [rad]
    # By testing, the max velocity in Gear 0 is around 1.7 m/s.
    # The max velocity in Gear 2 is around 3.6 m/s.
    MAX_SPEED_0 = 1.7               # [m/s]
    MAX_SPEED_1 = 3.6               # [m/s]
<<<<<<< HEAD
<<<<<<< HEAD
    BIAS_STEERING = -9

    is_sim = rx.Parameter(False)
=======
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
=======
>>>>>>> b921c25 (Added rmw-zenoh in dockerfile, added svea_example)

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
<<<<<<< HEAD
        if self.is_sim:
            self.BIAS_STEERING = 0

    def loop(self):
        """Main loop to publish control messages."""
        self.steering_msg.data = int(self.latest_controls[Controls.STEERING] * - 1.27 - self.BIAS_STEERING)  # 1.27 is a factor to convert from percent to the range of -127 to 127
=======

    def loop(self):
        """Main loop to publish control messages."""
        self.steering_msg.data = int(self.latest_controls[Controls.STEERING] * - 1.27)  # 1.27 is a factor to convert from percent to the range of -127 to 127
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
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
=======
    # scaling factor, percentage to lli actuation
    PERC_TO_LLI_COEFF = 1.27
    # An approximation of the range of velocity inputs around zero
    # which will not be enough to make the car move.
    VELOCITY_DEADBAND = 15.0        # [%]

    ## Binary masks ##

    # Bit indicating status of the transmission
    TRANSMISSION_MASK = 0b00000001
    # Bit indicating status of the forward differential
    FDIFF_MASK = 0b00000010
    # Bit indicating status of the rear differential
    RDIFF_MASK = 0b00000100
    # Bit indicating software idle
    SOFTWARE_IDLE_MASK = 0b00000001
    # Bit indicating remote idle
    REMOTE_IDLE_MASK = 0b00000010
    # Bit indicating remote override
    REMOTE_OVERRIDE_MASK = 0b00000100
    # Bit indicating emergency stop engaged
    EMERGENCY_MASK = 0b00001000

    _ctrl_request_top   = 'lli/ctrl_request' 
    _ctrl_actuated_top  = 'lli/ctrl_actuated' 

    def __init__(self, node: Node, log_length: int = 100) -> None:  
        
        self._node = node
        
        self._previous_velocity = None
        self._is_reverse = False

        self.ctrl_request = _ControlRequest() # Stores the last controls sent
        self.ctrl_msg = LLIControl()  # Message to send to ROS
        self.last_ctrl_update = rclpy.clock.Clock().now().to_msg()

        self._is_stop = False
        self.is_emergency = False
        self.is_ready = False
        self._ready_event = Event()

        # log of control requests and control actuated
        self.ctrl_request_log = deque(maxlen=log_length)
        self.ctrl_actuated_log = deque(maxlen=log_length)

    def start(self, wait=True) -> Self:
        """Spins up ROS background thread; must be called to start receiving
        and sending data.

        Args:
            wait: True if the interface should call `wait_until_ready` before
                returning.
        """
        self._node.get_logger().info('Starting Control Interface Node...')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        
        ## Publishers ##

        self._publish_ctrl_request = self._node.create_publisher(LLIControl,
                                                                 self._ctrl_request_top,
                                                                 qos_profile=qos_profile)

        ## Subscribers ##
        
        self._node.create_subscription(LLIControl, 
                                       self._ctrl_actuated_top,
                                       self._read_ctrl_actuated, 
                                       qos_profile=qos_profile)
        
        # duration = rclpy.duration.Duration(seconds=0.1)
        # rclpy.clock.Clock().sleep_for(duration)
        
        if wait:
            self.wait(10.0)

        if not self.is_ready:
            self._node.get_logger().info("LLI interface not responding during start of "
                                         "Control Interface. Setting ready anyway.")
        self.is_ready = True

        self._node.get_logger().info("Control Interface ready...") 
        return self

    def wait(self, timeout: Optional[None] = None):
        num_attempts = 0
        attempt_limit = int(timeout) or 1
        part_timeout = 1.0 if attempt_limit >= 1 else timeout
        self._ready_event.clear()
        while (not self.is_ready and
               rclpy.ok() and
               num_attempts < attempt_limit):
            num_attempts += 1
            self.send_control(ctrl_code=num_attempts)
            self.is_ready = self._ready_event.wait(part_timeout)

    def _read_ctrl_actuated(self, msg):
        self._is_reverse = self._detect_reverse_state(msg)
        self.ctrl_actuated_log.append(msg)
        self._ready_event.set()

    def _detect_reverse_state(self, msg):
        dead_zone = 2 # velocities with abs values <= than this = 0 to ESC
        velocity = msg.velocity
        try:
            if velocity > dead_zone or self._previous_velocity is None:
                return False
            elif (self._previous_velocity < dead_zone and
                  abs(velocity) <= dead_zone):
                return True
            else:
                return self._is_reverse
        finally:
            self._previous_velocity = velocity

    def _build_param_printout(self):
        # collect important params
        steering = self.ctrl_request.steering
        velocity = self.ctrl_request.velocity
        transmission = self.ctrl_request.transmission
        differential_front = self.ctrl_request.differential_front
        differential_rear = self.ctrl_request.differential_rear
        ctrl_code = self.ctrl_request.ctrl_code

        return  ("## Vehicle: {0}\n".format(self.vehicle_name)
                +"  -ctrl request:\n"
                +"      steering[%] - {0}\n".format(steering)
                +"      velocity[%] - {0}\n".format(velocity)
                +"      trans       - {0}\n".format(transmission)
                +"      diff_front  - {0}\n".format(differential_front)
                +"      diff_rear   - {0}\n".format(differential_rear)
                +"      ctrl_code   - {0}\n".format(ctrl_code)
                +"  -Is stopped:   {0}\n".format(self.is_stop)
                +"  -In reverse:   {0}\n".format(self._is_reverse)
                +"  -Gear:         {0}\n".format(self.gear)
                +"  -Diff_front:   {0}\n".format(self.differential_front)
                +"  -Diff_rear:    {0}\n".format(self.differential_rear)
                +"  -SW idle:      {0}\n".format(self.software_idle)
                +"  -Remote idle:  {0}\n".format(self.remote_idle)
                +"  -Rem override: {0}\n".format(self.remote_override)
                +"  -SW emergency: {0}\n".format(self.emergency))

    def __repr__(self):
        return self._build_param_printout()

    def __str__(self):
        return self._build_param_printout()

    def _steer_to_percent(self, steering):
        """Convert radians to percent of max steering actuation"""
        steering = float(steering)
        steer_percent = steering*100.0/self.MAX_STEERING_ANGLE
        return steer_percent

    def _vel_to_percent(self, velocity):
        velocity = float(velocity)
        vel_percent = velocity/self.max_speed * 100
        return int(vel_percent)

    def _clip_ctrl(self, steer_percent, vel_percent):
        clipped_steering = self._clip_steering(steer_percent)
        clipped_velocity = self._clip_velocity(vel_percent)
        return clipped_steering, clipped_velocity

    def _clip_steering(self, steer_percent):
        return min(self.MAX_STEER_PERCENT,
                               max(-self.MAX_STEER_PERCENT, steer_percent))

    def _clip_velocity(self, vel_percent):
        return min(self.MAX_SPEED_PERCENT,
                               max(-self.MAX_SPEED_PERCENT, vel_percent))

    def _remove_velocity_deadzone(self, vel_percent):
        deadband_coefficient = (100-self.VELOCITY_DEADBAND)/100
        deadband_term = cmp(vel_percent, 0)*self.VELOCITY_DEADBAND
        if abs(vel_percent) > 0.05:
            vel_percent = vel_percent*deadband_coefficient + deadband_term
        else:
            vel_percent = 0
        return vel_percent

    def _set_reverse(self, reverse):
        if self._is_reverse == reverse or self.is_stop:
            return
        if self._previous_velocity is None:
            current_velocity = 0
        else:
            current_velocity = self._previous_velocity
        reverse_msg = LLIControl()
        reverse_msg.steering = -128
        if reverse:
            reverse_msg.velocity = -15
            self._publish_ctrl_request.publish(reverse_msg)
            duration = rclpy.duration.Duration(seconds=0.05)
            rclpy.clock.Clock().sleep_for(duration)
            reverse_msg.velocity = 0
            self._publish_ctrl_request.publish(reverse_msg)
            duration = rclpy.duration.Duration(seconds=0.05)
            rclpy.clock.Clock().sleep_for(duration)
        else:
            reverse_msg.velocity = 15
            self._publish_ctrl_request.publish(reverse_msg)
            duration = rclpy.duration.Duration(seconds=0.05)
            rclpy.clock.Clock().sleep_for(duration)
            reverse_msg.velocity = current_velocity

    def send_control(
        self,
        steering: Optional[float] = None,
        velocity: Optional[float] = None,
        brake_force: float = 0,
        transmission: int = -1,
        differential_front: int = -1,
        differential_rear: int = -1,
        ctrl_code: int = 0,
    ):
        """Method for taking control inputs and implementing features over the
        control inputs.

        This method converts steering angles and velocities from `[rad]` and
        `[m/s]` to unit-less values that the low-level system expects,
        saturates the unit-less values, implements a stopping feature for
        blocking control inputs (note, this is not necessarily a braking
        feature, it only blocks inputs from being published, thus it should not
        be used for emergency braking), and sends/publishes inputs to the
        low-level interface.

        If an argument is left empty the low-level interface will be told to
        use the last sent value. The same is true if the gear or differential
        arguments have any other values than 0 or 1. If you do *not* call
        `send_control` then the car will *not* do anything.

        Args:
            steering: Input steering angle for the car in `[rad]`, if argument
                left empty the low-level system will use the last sent valid
                value.
            velocity: Input velocity for the car `[m/s]`, if argument left
                empty the low-level system will implement the last sent valid
                value.
            brake_force: Brake force as a percentage `[0, 100]` of maximum
                braking force.
            transmission: 0 means low gear, 1 means high gear, -1 means keep
                the currently set gear.
            differential_front: 0 means unlocked, 1 means locked, -1 means keep
                the currently set lock state.
            differential_rear: 0 means unlocked, 1 means locked, -1 means keep
                the currently set lock state.
            ctrl_code: Deprecated.
        """

        # Steering
        if steering is not None and not isnan(steering):
            steer_percent = self._steer_to_percent(steering)
            steer_percent = self._clip_steering(steer_percent)
            self.ctrl_request.steering = steer_percent
            self.ctrl_msg.steering = \
                    round(steer_percent * self.PERC_TO_LLI_COEFF)
        else:
            self.ctrl_msg.steering = -128

        # Velocity
        if brake_force > 0:
            if self._is_reverse:
                self._set_reverse(False)
            self.ctrl_msg.velocity = \
                    -round(brake_force * self.PERC_TO_LLI_COEFF)
            self.ctrl_request.velocity = 0
        elif velocity is not None and not isnan(velocity):
            vel_percent = self._vel_to_percent(velocity)
            vel_percent = self._clip_velocity(vel_percent)
            vel_percent = self._remove_velocity_deadzone(vel_percent)
            self.ctrl_request.velocity = vel_percent
            self.ctrl_msg.velocity = round(vel_percent * self.PERC_TO_LLI_COEFF)
            if velocity < 0 and not self._is_reverse:
                self._set_reverse(True)
        else:
            self.ctrl_msg.velocity = -128

        # Transmission and differentials
        self.ctrl_msg.trans_diff = 0
        if transmission in (0, 1):
            self.ctrl_request.transmission = transmission
            self.ctrl_msg.trans_diff ^= (1 << 3 ^ transmission << 0)
        if differential_front in (0, 1):
            self.ctrl_request.differential_front = differential_front
            self.ctrl_msg.trans_diff ^= (1 << 4 ^ differential_front << 1)
        if differential_front in (0, 1):
            self.ctrl_request.differential_rear = differential_rear
            self.ctrl_msg.trans_diff ^= (1 << 5 ^ differential_rear << 2)
        self.ctrl_msg.ctrl = ctrl_code
        self.ctrl_request.ctrl_code = ctrl_code

        if not self.is_stop:
            self._publish_ctrl_request.publish(self.ctrl_msg)
            self.ctrl_request_log.append(self.ctrl_request)

    @property
    def is_stop(self) -> bool: return self._is_stop

    @is_stop.setter
    def is_stop(self, is_stop: bool):
        """Setter function for stopping the car.

        **This is not the same as emergency braking.**

        Args:
            is_stop: Flag for blocking or unblocking the control inputs to the
                car.
        """
        self._is_stop = is_stop
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)

    @property
    def max_speed(self) -> float:
        """Get the maximum speed, dependent on gear.

        Returns:
            The maximum speed, independent of direction
        """
<<<<<<< HEAD
        return self.MAX_SPEED_1 if self.highgear_msg.data else self.MAX_SPEED_0
=======
        return self.MAX_SPEED_1 if self.gear else self.MAX_SPEED_0

    @property
    def max_speed_signal(self) -> float:
        """Get the maximal velocity that the controller will actually attempt
        to actuate for the current gear.

        Returns:
            The maximum actuated speed, independent of direction.
        """
        return self.max_speed * self.MAX_SPEED_PERCENT/100

    @property
    def gear(self) -> int:
        """Current gear.

        Returns:
            `0` if low gear `1` if high gear, `None` if no information has been
            received.
        """
        try:
            trans_diff = self.ctrl_actuated_log[-1].trans_diff
            return trans_diff & self.TRANSMISSION_MASK
        except IndexError:
            return None

    @property
    def differential_front(self):
        """ Status of the front differential lock

        :return: `0` if unlocked `1` if locked,
                 `None` if no information has been received.
        :rtype: int
        """
        try:
            trans_diff = self.ctrl_actuated_log[-1].trans_diff
            return trans_diff & self.FDIFF_MASK
        except IndexError:
            return None

    @property
    def differential_rear(self):
        """ Status of the rear differential lock

        :return: `0` if unlocked `1` if locked,
                 `None` if no information has been received.
        :rtype: int
        """
        try:
            trans_diff = self.ctrl_actuated_log[-1].trans_diff
            return trans_diff & self.RDIFF_MASK
        except IndexError:
            return None

    @property
    def software_idle(self):
        """ Check if the computer is considered idle by the low level
        interface.

        :return: `True` if idle, `False` if active,
                 `None` if no information has been received.
        :rtype: bool
        """
        try:
            ctrl_code = self.ctrl_actuated_log[-1].ctrl
            return bool(ctrl_code & self.SOFTWARE_IDLE_MASK)
        except IndexError:
            return None

    @property
    def remote_idle(self):
        """ Check if the remote is considered idle by the low level
        interface.

        :return: `True` if idle, `False` if active,
                 `None` if no information has been received.
        :rtype: bool
        """
        try:
            ctrl_code = self.ctrl_actuated_log[-1].ctrl
            return bool(ctrl_code & self.REMOTE_IDLE_MASK)
        except IndexError:
            return None

    @property
    def remote_override(self):
        """ Check if the remote override is active. If the override is
        active the control requests will be ignored by the low level
        interface.

        :return: `True` if override is engaged, `False` otherwise,
                 `None` if no information has been received.
        :rtype: bool
        """
        try:
            ctrl_code = self.ctrl_actuated_log[-1].ctrl
            return bool(ctrl_code & self.REMOTE_OVERRIDE_MASK)
        except IndexError:
            return None

    @property
    def emergency(self):
        """Check if the emergency flag is set. If the emergency flag can
        not be cleared from ROS it is also possible to clear it by not
        sending any control requests for at least 5 seconds, and setting
        the remote to override.

        :return: `True` if emergency flag is set, `False` otherwise,
                 `None` if no information has been received.
        :rtype: bool
        """
        try:
            ctrl_code = self.ctrl_actuated_log[-1].ctrl
            return bool(ctrl_code & self.EMERGENCY_MASK)
        except IndexError:
            return None

class _ControlRequest(object):
    def __init__(self):
        self.steering = 0.0
        self.velocity = 0.0
        self.transmission = 0
        self.differential_front = False
        self.differential_rear = False
        self.ctrl_code = 0
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)

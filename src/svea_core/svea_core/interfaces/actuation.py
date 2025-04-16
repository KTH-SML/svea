"""
Author: Frank Jiang, Tobias Bolin
"""
from threading import Event
from collections import deque
from math import pi, isnan
from typing import Optional, Self

import rclpy
import rclpy.clock
import rclpy.duration
from rclpy.node import Node
from svea_msgs.msg import LLIControl as lli_ctrl
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

__all__ = [
    'ActuationInterface',
]

def cmp(a, b):
    return (a > b) - (a < b)


class ActuationInterface[N:Node]:
    """Interface object for sending actuation commands to the SVEA car's low-level
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
    # saturation input limits to keep hardware healthy
    MAX_STEER_PERCENT = 90          # [%]
    MAX_SPEED_PERCENT = 90          # [%]
    # assumed maximum steering angle, approximately 40 degrees
    MAX_STEERING_ANGLE = 40*pi/180  # [rad]
    # By testing, the max velocity in Gear 0 is around 1.7 m/s.
    # The max velocity in Gear 2 is around 3.6 m/s.
    MAX_SPEED_0 = 1.7               # [m/s]
    MAX_SPEED_1 = 3.6               # [m/s]
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

    def __init__(self, node: N, log_length: int = 100) -> None:  
        self.node = node
        self._request_topic = 'lli/ctrl_request' 
        self._actuated_topic = 'lli/ctrl_actuated' 
        self._remote_topic = 'lli/remote' 
       
        
        self._previous_velocity = None
        self._is_reverse = False

        self.ctrl_request = _ControlRequest() # Stores the last controls sent
        self.ctrl_msg = lli_ctrl()  # Message to send to ROS
        self.last_ctrl_update = rclpy.clock.Clock().now().to_msg()

        self._is_stop = False
        self.is_emergency = False
        self.is_ready = False
        self._ready_event = Event()

        # log of control requests and control actuated
        self.ctrl_request_log = deque(maxlen=log_length)
        self.ctrl_actuated_log = deque(maxlen=log_length)
        self.remote_log = deque(maxlen=log_length)

    def start(self) -> Self:
        """Spins up ROS background thread; must be called to start receiving
        and sending data.

        Args:
            wait: True if the interface should call `wait_until_ready` before
                returning.
        """
        self.node.get_logger().info('Starting Control Interface Node...')
        self.node_name = 'control_interface'
        self._start_publish()
        self._start_listen()
        duration = rclpy.duration.Duration(seconds=0.1)
        rclpy.clock.Clock().sleep_for(duration)
        self._wait_until_ready()
        if not self.is_ready:
            self.node.get_logger().info("LLI interface not responding during start of "
                                            "Control Interface. Seting ready anyway.")
        self.is_ready = True
        self.node.get_logger().info("Control Interface ready...") 

        return self


    def _wait_until_ready(self, timeout: float = 10.0) -> bool:
        """Internal method for waiting until the Control Interface is ready.

        Args:
            timeout: Number of seconds to wait for a response from the low
                level interface.

        Returns:
            False if timed out or rospy is shutdown, true otherwise. Will
            return when the interface is ready, after `timeout` seconds or if
            rospy is shutdown.
        """
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
        return self.is_ready

    def _start_listen(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSReliabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1,)
        
        self.node.create_subscription(lli_ctrl, 
                                      self._actuated_topic,
                                      self._read_ctrl_actuated, 
                                      qos_profile=qos_profile)
        
        self.node.create_subscription(lli_ctrl, 
                                      self._remote_topic,
                                      self._read_remote,
                                      qos_profile=qos_profile)

    def _start_publish(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSReliabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1,)
        
        self.ctrl_request_pub = self.node.create_publisher(lli_ctrl,
                                                           self._request_topic,
                                                           qos_profile=qos_profile)

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

    def _read_remote(self, msg):
        self.remote_log.append(msg)

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
        reverse_msg = lli_ctrl()
        reverse_msg.steering = -128
        if reverse:
            reverse_msg.velocity = -15
            self.ctrl_request_pub.publish(reverse_msg)
            duration = rclpy.duration.Duration(seconds=0.05)
            rclpy.clock.Clock().sleep_for(duration)
            reverse_msg.velocity = 0
            self.ctrl_request_pub.publish(reverse_msg)
            duration = rclpy.duration.Duration(seconds=0.05)
            rclpy.clock.Clock().sleep_for(duration)
        else:
            reverse_msg.velocity = 15
            self.ctrl_request_pub.publish(reverse_msg)
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
            self.ctrl_request_pub.publish(self.ctrl_msg)
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

    @property
    def max_speed(self) -> float:
        """Get the maximum speed, dependent on gear.

        Returns:
            The maximum speed, independent of direction
        """
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

#! /usr/bin/env python3

"""
single_encoder_twist_filter

Turns the unsigned cumulative distance published by the mavros wheel_odometry
plugin (one encoder per wheel) into a directional
geometry_msgs/TwistWithCovarianceStamped for robot_localization.

This is a drop-in replacement for mavros/wheel_odometry/velocity: same message
type, same semantics, but the sign of the motion is restored. Nothing is
integrated, so a momentary direction error only affects that one sample instead
of accumulating into a pose.

------------------------------------------------------------------------------
Where the direction comes from
------------------------------------------------------------------------------
The encoders only report how far each wheel has turned, never which way, so the
sign has to come from the command side. Two sources are combined, and which one
applies is decided by the override mode channel on the receiver:

    channels[rc_mode_channel]           active source
    ---------------------------------   -------------------------------------
    ~2000  rc + manual                  RC stick if it is off centre,
                                        otherwise mavros/manual_control/send
    ~1500  rc only                      RC stick
    ~1000  no control                   neither; hold the previous direction

    channels[rc_throttle_channel]       1500 centre, 2000 full forward,
                                        1000 full reverse
    mavros/manual_control/send .z       z = 500 - velocity_percent*5,
                                        so z<500 forward, z>500 reverse

Only the sign relative to centre is used, so the exact full-scale endpoints do
not matter.

------------------------------------------------------------------------------
Coast-down hysteresis
------------------------------------------------------------------------------
A command is an intent, not an observation. When the throttle flips from
forward to reverse the car keeps rolling forward for a few hundred milliseconds,
and flipping the sign immediately would report that roll-out as reverse travel.
The sign is therefore only flipped once the measured wheel speed has dropped
below reverse_speed_threshold. The cost is a short lag at the start of a genuine
reversal, which is much smaller than the error it prevents.

------------------------------------------------------------------------------
Model
------------------------------------------------------------------------------
    v = (ds_R + ds_L) / (2 * dt)
    w = (ds_R - ds_L) / (axle_track * dt)
"""

from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from rclpy.time import Time

from geometry_msgs.msg import TwistWithCovarianceStamped
from mavros_msgs.msg import WheelOdomStamped, ManualControl, RCIn

from svea_core import rosonic as rx


# mavros publishes with sensor-data QoS, so subscribers must be BEST_EFFORT.
qos_subber = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=20,
)

# robot_localization subscribes with RELIABLE, so this publisher must match.
qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Override modes decoded from the mode channel.
MODE_NONE = 'no_control'
MODE_RC = 'rc_only'
MODE_BOTH = 'rc_and_manual'


class single_encoder_twist_filter(rx.Node):
    """Unsigned cumulative distance + commanded direction -> body-frame twist."""

    ## ---------------- Parameters ---------------- ##

    # -- Topics --
    distance_topic = rx.Parameter('mavros/wheel_odometry/distance')
    twist_topic = rx.Parameter('wheel_odometry/twist/filtered')
    control_topic = rx.Parameter('mavros/manual_control/send')
    rc_topic = rx.Parameter('mavros/rc/in')

    # Twist is expressed in the body frame, so this is the message frame_id.
    base_frame = rx.Parameter('base_link')

    # -- Vehicle geometry (see svea_core/params/mavros.yaml) --
    axle_track = rx.Parameter(0.32)
    left_index = rx.Parameter(0)
    right_index = rx.Parameter(1)

    # -- RC receiver --
    rc_throttle_channel = rx.Parameter(1)
    rc_mode_channel = rx.Parameter(4)

    # The nominal centre. Receivers vary by a few tens of counts and drift with
    # temperature, so this is only a starting point and a sanity reference --
    # see rc_neutral_auto below.
    rc_neutral = rx.Parameter(1513)
    rc_deadband = rx.Parameter(1)
    rc_reversed = rx.Parameter(False)   # True if stick forward reads below centre
    rc_timeout = rx.Parameter(0.5)      # receiver considered stale after this [s]

    # Learn the true stick centre instead of trusting rc_neutral. The first
    # rc_calib_samples readings are assumed to be taken with the stick centred
    # and their median becomes the estimate; afterwards it is nudged slowly
    # whenever a reading falls inside the deadband.
    rc_neutral_auto = rx.Parameter(True)
    rc_calib_samples = rx.Parameter(50)
    rc_neutral_adapt_alpha = rx.Parameter(0.01)

    # An estimate further than this from rc_neutral is rejected. Guards against
    # calibrating on a failsafe value when the transmitter is off at startup.
    rc_neutral_max_offset = rx.Parameter(200)

    # Mode channel thresholds. A reading at or above mode_both_min means
    # rc + manual, at or above mode_rc_min means rc only, below that no control.
    mode_both_min = rx.Parameter(1750)
    mode_rc_min = rx.Parameter(1250)

    # -- Manual control (ManualControl.z) --
    control_neutral = rx.Parameter(500.0)
    control_deadband = rx.Parameter(15.0)   # about 3 percent throttle
    control_timeout = rx.Parameter(0.5)

    # -- Direction handling --
    default_direction = rx.Parameter(1)     # used before any command arrives

    # Do not flip the sign until wheel speed has dropped below this [m/s].
    reverse_speed_threshold = rx.Parameter(0.05)

    # -- Sanity limits --
    max_dt = rx.Parameter(0.5)
    max_wheel_step = rx.Parameter(0.5)

    # -- Output --
    linear_covariance = rx.Parameter(0.05)
    angular_covariance = rx.Parameter(0.2)

    ## ---------------- State ---------------- ##

    _last_dist = None
    _last_time = None

    _dir = None             # sign currently applied to the encoder increments

    _rc_throttle = None     # last raw throttle channel reading
    _rc_mode = None         # last raw mode channel reading
    _rc_stamp = None

    _rc_calib = None        # samples collected during startup calibration
    _rc_neutral_est = None  # estimated stick centre

    _mc_z = None            # last raw ManualControl.z
    _mc_stamp = None

    _warned_len = False
    _warned_channels = False
    _last_mode = None

    ## ---------------- Publishers ---------------- ##

    twist_pub = rx.Publisher(TwistWithCovarianceStamped, twist_topic, qos_pubber)

    ## ---------------- Subscribers ---------------- ##

    @rx.Subscriber(RCIn, rc_topic, qos_subber)
    def rc_cb(self, msg):
        """Cache the raw throttle and mode channels."""

        tc = int(self.rc_throttle_channel)
        mc = int(self.rc_mode_channel)

        if len(msg.channels) <= max(tc, mc):
            if not self._warned_channels:
                self.get_logger().error(
                    f'RCIn has {len(msg.channels)} channels, cannot index '
                    f'{tc}/{mc}. Check rc_throttle_channel and rc_mode_channel.'
                )
                self._warned_channels = True
            return

        self._rc_throttle = int(msg.channels[tc])
        self._rc_mode = int(msg.channels[mc])
        self._rc_stamp = self._now()

        self._update_neutral(self._rc_throttle)

    @rx.Subscriber(ManualControl, control_topic, qos_subber)
    def control_cb(self, msg):
        """Cache the commanded throttle from the autonomous controller."""

        self._mc_z = float(msg.z)
        self._mc_stamp = self._now()

    @rx.Subscriber(WheelOdomStamped, distance_topic, qos_subber)
    def distance_cb(self, msg):

        dist = list(msg.data)

        li = int(self.left_index)
        ri = int(self.right_index)

        if len(dist) <= max(li, ri):
            if not self._warned_len:
                self.get_logger().error(
                    f'Distance array has length {len(dist)}, cannot index {li}/{ri}. '
                    f'Check the count / send_raw settings of the mavros '
                    f'wheel_odometry plugin.'
                )
                self._warned_len = True
            return

        t = Time.from_msg(msg.header.stamp).nanoseconds * 1.0e-9

        if self._last_dist is None:
            self._last_dist = dist
            self._last_time = t
            self.get_logger().info('First wheel distance message received, baseline set')
            return

        dt = t - self._last_time
        d_l = dist[li] - self._last_dist[li]
        d_r = dist[ri] - self._last_dist[ri]

        self._last_dist = dist
        self._last_time = t

        if dt <= 0.0 or dt > self.max_dt:
            self.get_logger().warn(f'Invalid dt={dt:.4f}, skipping this frame')
            return

        # The counters are monotonic, so a negative or oversized increment means
        # they were reset.
        step = self.max_wheel_step
        if d_l < -1.0e-6 or d_r < -1.0e-6 or abs(d_l) > step or abs(d_r) > step:
            self.get_logger().warn(
                f'Implausible increment (dL={d_l:.4f}, dR={d_r:.4f}), '
                f'treating as a counter reset and skipping'
            )
            return

        # Unsigned wheel speed, used to gate the sign flip.
        speed = 0.5 * (d_l + d_r) / dt

        # Restore the sign.
        sign = float(self._resolve_direction(speed))
        d_l *= sign
        d_r *= sign

        v = 0.5 * (d_l + d_r) / dt
        w = (d_r - d_l) / (self.axle_track * dt)

        self._publish(msg.header.stamp, v, w)

    ## ---------------- Direction ---------------- ##

    def _now(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _rc_alive(self):
        return (self._rc_stamp is not None
                and self._now() - self._rc_stamp < self.rc_timeout)

    def _mc_alive(self):
        return (self._mc_stamp is not None
                and self._now() - self._mc_stamp < self.control_timeout)

    def _mode(self):
        """Decode the override mode channel.

        Falls back to rc + manual when the receiver is stale, so that the
        autonomous command still provides a direction if RC drops out.
        """
        if not self._rc_alive() or self._rc_mode is None:
            return MODE_BOTH
        if self._rc_mode >= self.mode_both_min:
            return MODE_BOTH
        if self._rc_mode >= self.mode_rc_min:
            return MODE_RC
        return MODE_NONE

    def _update_neutral(self, value):
        """Track the true stick centre.

        The nominal centre is only approximate, so it is calibrated from the
        first samples after startup and then adapted slowly. Adaptation only
        happens for readings already inside the deadband, which keeps a held
        stick from being learned as the centre.
        """

        nominal = int(self.rc_neutral)
        max_off = int(self.rc_neutral_max_offset)

        if not self.rc_neutral_auto:
            self._rc_neutral_est = float(nominal)
            return

        # Startup calibration: median of the first batch of samples.
        if self._rc_calib is not None:
            self._rc_calib.append(value)
            if len(self._rc_calib) < int(self.rc_calib_samples):
                return

            est = float(sorted(self._rc_calib)[len(self._rc_calib) // 2])
            self._rc_calib = None

            if abs(est - nominal) > max_off:
                self.get_logger().warn(
                    f'Calibrated centre {est:.0f} is more than {max_off} from '
                    f'the nominal {nominal}; the stick was probably not centred '
                    f'or the transmitter was off. Falling back to {nominal}.'
                )
                self._rc_neutral_est = float(nominal)
            else:
                self._rc_neutral_est = est
                self.get_logger().info(f'RC throttle centre calibrated at {est:.0f}')
            return

        # Slow adaptation, only inside the deadband.
        if abs(value - self._rc_neutral_est) <= self.rc_deadband:
            a = float(self.rc_neutral_adapt_alpha)
            updated = (1.0 - a) * self._rc_neutral_est + a * value
            if abs(updated - nominal) <= max_off:
                self._rc_neutral_est = updated

    def _rc_direction(self):
        """Sign of the RC throttle stick, or None when centred or stale."""
        if not self._rc_alive() or self._rc_throttle is None:
            return None
        if self._rc_neutral_est is None:
            return None     # still calibrating
        delta = self._rc_throttle - self._rc_neutral_est
        if self.rc_reversed:
            delta = -delta
        if abs(delta) <= self.rc_deadband:
            return None
        return 1 if delta > 0.0 else -1

    def _manual_direction(self):
        """Sign of the commanded throttle, or None when centred or stale.

        ManualControl.z is inverted: z = 500 - velocity_percent*5.
        """
        if not self._mc_alive() or self._mc_z is None:
            return None
        delta = self._mc_z - float(self.control_neutral)
        if abs(delta) <= self.control_deadband:
            return None
        return 1 if delta < 0.0 else -1

    def _commanded_direction(self):
        """Direction implied by whichever source the mode channel selects.

        Returns None when nothing is commanding motion, which means the
        previous direction should be held.
        """

        mode = self._mode()

        if mode != self._last_mode:
            self.get_logger().info(f'Override mode: {mode}')
            self._last_mode = mode

        if mode == MODE_NONE:
            return None

        rc_dir = self._rc_direction()

        if mode == MODE_RC:
            return rc_dir

        # MODE_BOTH: a stick input means the operator is intervening, so it
        # wins over whatever the controller is sending.
        return rc_dir if rc_dir is not None else self._manual_direction()

    def _resolve_direction(self, speed):
        """Pick the sign to apply to this sample."""

        cmd_dir = self._commanded_direction()

        if cmd_dir is None:
            # Nothing commanding motion: keep rolling in the current direction.
            return self._dir if self._dir is not None else int(self.default_direction)

        if self._dir is None:
            self._dir = cmd_dir
            return self._dir

        # Wait for the coast-down before honouring a reversal.
        if cmd_dir != self._dir and speed < self.reverse_speed_threshold:
            self.get_logger().info(f'Direction: {self._dir:+d} -> {cmd_dir:+d}')
            self._dir = cmd_dir

        return self._dir

    ## ---------------- Output ---------------- ##

    def _publish(self, stamp, v, w):

        msg = TwistWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        msg.twist.covariance[0] = float(self.linear_covariance)    # vx
        msg.twist.covariance[7] = 1.0e6                            # vy
        msg.twist.covariance[14] = 1.0e6                           # vz
        msg.twist.covariance[21] = 1.0e6                           # roll rate
        msg.twist.covariance[28] = 1.0e6                           # pitch rate
        msg.twist.covariance[35] = float(self.angular_covariance)  # yaw rate

        self.twist_pub.publish(msg)

    ## ---------------- Lifecycle ---------------- ##

    def on_startup(self):

        if self.mode_rc_min >= self.mode_both_min:
            raise ValueError('mode_rc_min must be below mode_both_min')

        self._last_dist = None
        self._last_time = None
        self._dir = None

        self._rc_throttle = None
        self._rc_mode = None
        self._rc_stamp = None

        self._rc_calib = [] if self.rc_neutral_auto else None
        self._rc_neutral_est = None if self.rc_neutral_auto else float(self.rc_neutral)

        self._mc_z = None
        self._mc_stamp = None

        self._last_mode = None

        self.get_logger().info(
            f'single_encoder_twist_filter started\n'
            f'  distance : {self.distance_topic}\n'
            f'  rc       : {self.rc_topic} '
            f'(throttle ch {self.rc_throttle_channel}, mode ch {self.rc_mode_channel})\n'
            f'  manual   : {self.control_topic}\n'
            f'  twist    : {self.twist_topic}\n'
            f'  frame    : {self.base_frame}\n'
            f'  track    : {self.axle_track} m'
        )

    @rx.Timer(5.0)
    def watchdog(self):
        if not self._rc_alive() and not self._mc_alive():
            self.get_logger().warn(
                'Neither RC nor manual control is publishing; direction is '
                'being held at its last value'
            )


if __name__ == '__main__':
    single_encoder_twist_filter.main()

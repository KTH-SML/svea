#! /usr/bin/env python3

import math

from svea_core import rosonic as rx
from svea_core.interfaces import ActuationInterface
from geometry_msgs.msg import Twist


class twist_consumer(rx.Node):
    """Twist (v, omega) -> SVEA actuation (steering angle, velocity)."""

    twist_top  = rx.Parameter('cmd_vel')
    twist_type = rx.Parameter('geometry_msgs/msg/Twist')

    wheelbase    = rx.Parameter(0.324)   # [m]
    max_steering = rx.Parameter(0.524)   # [rad]
    max_velocity = rx.Parameter(0.8)     # [m/s]
    min_speed_for_steering = rx.Parameter(0.05)

    velocity_sign = rx.Parameter(1.0)
    steering_sign = rx.Parameter(1.0)

    cmd_timeout  = rx.Parameter(0.6)     # [s]
    use_difflock = rx.Parameter(True)

    actuation = ActuationInterface()

    _velocity = 0.0
    _steering = 0.0
    _last_msg = None

    def on_startup(self):
        if self.twist_type not in ("geometry_msgs/msg/Twist",
                                   "geometry_msgs/msg/TwistStamped"):
            raise TypeError(f'Invalid message type for {self.twist_top}: {self.twist_type}')

        if self.use_difflock:
            self.actuation.enable_difflock()

        self._last_msg = self.get_clock().now()
        self.get_logger().info(
            f'twist_consumer listening on "{self.twist_top}" ({self.twist_type})')

    def _omega_to_steering(self, v, w):
        """Ackermann inverse kinematics: delta = atan(L * omega / v)."""
        if abs(v) < self.min_speed_for_steering:
            return 0.0
        delta = math.atan(self.wheelbase * w / abs(v))
        return max(-self.max_steering, min(self.max_steering, delta))

    @rx.Subscriber(twist_type, twist_top)
    def twist_cb(self, msg):
        tw = msg if isinstance(msg, Twist) else msg.twist

        v = float(tw.linear.x)
        w = float(tw.angular.z)
        v = max(-self.max_velocity, min(self.max_velocity, v))

        self._velocity = v
        self._steering = self._omega_to_steering(v, w)
        self._last_msg = self.get_clock().now()

    @rx.Timer(0.1)
    def loop(self):
        age = (self.get_clock().now() - self._last_msg).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self._velocity = 0.0
            self._steering = 0.0

        self.actuation.send_control(self._steering * self.steering_sign,
                                    -1 * self._velocity * self.velocity_sign)


if __name__ == '__main__':
    twist_consumer.main()

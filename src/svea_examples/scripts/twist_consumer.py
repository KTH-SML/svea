#! /usr/bin/env python3

from svea_core import rosonic as rx
from svea_core.interfaces import ActuationInterface
from geometry_msgs.msg import Twist

class twist_consumer(rx.Node):
    """Teleoperation control node for SVEA."""

    twist_top = rx.Parameter('/cmd_vel')
    twist_type = rx.Parameter('geometry_msgs/msg/Twist')

    actuation = ActuationInterface()

    _velocity = 0.0
    _steering = 0.0

    def on_startup(self):
        if self.twist_type not in ("geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped"):
            raise TypeError(f'Invalid message type for {self.twist_top}: {self.twist_type}')

    @rx.Subscriber(twist_type, twist_top)
    def twist_cb(self, msg):
        if isinstance(msg, Twist):
            self._velocity = msg.linear.x
            self._steering = - msg.angular.z
        else:
            self._velocity = msg.twist.linear.x
            self._steering = - msg.twist.angular.z

    @rx.Timer(0.1)
    def loop(self):
        self.actuation.send_control(self._steering, self._velocity)

if __name__ == '__main__':
    twist_consumer.main()

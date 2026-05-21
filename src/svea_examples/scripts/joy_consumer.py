#! /usr/bin/env python3

from svea_core import rosonic as rx
from svea_core.interfaces import ActuationInterface
from sensor_msgs.msg import Joy
from math import pi
from dataclasses import dataclass


@dataclass
class Xbox360Controller:
    # Axes
    LSX: float = 0.
    LSY: float = 0.
    LT: float = 0.
    RSX: float = 0.
    RSY: float = 0.
    RT: float = 0.
    # Buttons
    A: bool = False
    B: bool = False
    X: bool = False
    Y: bool = False
    LB: bool = False
    RB: bool = False
    BACK: bool = False
    START: bool = False
    XBOX: bool = False
    LSB: bool = False
    RSB: bool = False
    DPADU: bool = False
    DPADD: bool = False
    DPADL: bool = False
    DPADR: bool = False

    @classmethod
    def from_joy(cls, msg):
        return cls(*msg.axes[:6], *msg.buttons[:15])


class joy_consumer(rx.Node):
    """Teleoperation control node for SVEA."""

    MAX_VELOCITY = 0.7
    MAX_STEERING = pi/6

    joy_top = rx.Parameter('/joy')
    joy_kind = rx.Parameter('xbox')

    actuation = ActuationInterface()
    
    _velocity = 0.0
    _steering = 0.0

    def on_startup(self):
        if self.joy_kind == 'xbox':
            self._previous = Xbox360Controller()
        else:
            raise NotImplementedError(f'Not Implemented: Joy kind: {self.joy_kind}')

    @rx.Subscriber(Joy, joy_top)
    def joy_cb(self, msg):
        if self.joy_kind == 'xbox':
            u = Xbox360Controller.from_joy(msg)
            ## Velocity: Right Trigger (RT) + Left Button (LB)
            self._velocity = (1 - u.RT)/2 * self.MAX_VELOCITY
            if msg.buttons[4]:
                self._velocity*= -1
            ## Steering: Left Stick
            self._steering = u.LSX * self.MAX_STEERING
            ## Gear: Right Bumper (B) + Left Bumper (X)
            # at release
            if self._previous.B and not u.B:
                self.actuation.enable_highgear()
            if self._previous.X and not u.X:
                self.actuation.disable_highgear()
            ## Diff: Top Bumper (Y)
            # at release
            if self._previous.Y and not u.Y:
                self.actuation.toggle_difflock()
        # Update history
        self._previous = u

    @rx.Timer(0.1)
    def loop(self):
        self.actuation.send_control(self._steering, self._velocity)

if __name__ == '__main__':
    joy_consumer.main()

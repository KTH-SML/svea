#! /usr/bin/env python3

from svea_core import rosonic as rx
from svea_core.interfaces import ActuationInterface
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from math import pi
from dataclasses import dataclass

class Joystick:

    @classmethod
    def AXES(cls):
        return [name 
                for name, T in cls.__annotations__.items()
                if T is float]

    @classmethod
    def BUTTONS(cls):
        return [name 
                for name, T in cls.__annotations__.items()
                if T is bool]

    @classmethod
    def CLS(cls): return cls

    @classmethod
    def from_msg(cls, msg):
        NAXES = len(cls.AXES())
        NBTNS = len(cls.BUTTONS())
        return cls(*msg.axes[:NAXES], *msg.buttons[:NBTNS])

@dataclass
class Xbox360Controller(Joystick):
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


class joy_consumer(rx.Node):
    """Teleoperation control node for SVEA."""

    MAX_VELOCITY = 0.7
    MAX_STEERING = pi/6

    joy_top = rx.Parameter('/joy')
    joy_kind = rx.Parameter('xbox')
    joy_btns = rx.Parameter('') # BTN:name,BTN:name

    actuation = ActuationInterface()
    
    _velocity = 0.0
    _steering = 0.0

    def on_startup(self):
        logger = self.get_logger()
        if self.joy_kind == 'xbox':
            self._previous = Xbox360Controller()
            unused_btns = set(Xbox360Controller.BUTTONS()) - set('LB B X Y'.split())
        else:
            raise NotImplementedError(f'Not Implemented: Joy kind: {self.joy_kind}')
        self._btns = {}
        if self.joy_btns:
            for pair in self.joy_btns.split(','):
                assert ':' in pair, f'Invalid pair mapping: {pair}'
                btn, name = pair.split(':')
                if btn in unused_btns:
                    self._btns[btn] = self.create_client(Empty, name)
                    while not self._btns[btn].wait_for_service(timeout_sec=1.0):
                        logger.info(f'Service "{name}" not available, keep waiting...')
                    logger.debug(f'Button {btn} bound to service "{name}"')

    @rx.Subscriber(Joy, joy_top)
    def joy_cb(self, msg):
        if self.joy_kind == 'xbox':
            joy = Xbox360Controller.from_msg(msg)
            ## Velocity: Right Trigger (RT) + Left Button (LB)
            self._velocity = (1 - joy.RT)/2 * self.MAX_VELOCITY
            if msg.buttons[4]:
                self._velocity*= -1
            ## Steering: Left Stick
            self._steering = joy.LSX * self.MAX_STEERING
            ## Gear: Right Bumper (B) + Left Bumper (X)
            # at release
            if self._previous.B and not joy.B:
                self.actuation.enable_highgear()
            if self._previous.X and not joy.X:
                self.actuation.disable_highgear()
            ## Diff: Top Bumper (Y)
            # at release
            if self._previous.Y and not joy.Y:
                self.actuation.toggle_difflock()
        ## Button Services
        for btn, cli in self._btns.items():
            if getattr(self._previous, btn) and not getattr(joy, btn):
                self.get_logger().warn(f'Button Press Event: {btn}')
                cli.call_async(Empty.Request())
        ## Update history
        self._previous = joy

    @rx.Timer(0.1)
    def loop(self):
        self.actuation.send_control(self._steering, self._velocity)

if __name__ == '__main__':
    joy_consumer.main()

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
    DPADL: bool = False
    DPADR: bool = False
    DPADU: bool = False
    DPADD: bool = False

@dataclass
class LogitechG27(Joystick):
    # Axes
    WHEEL: float = 0.
    LEFT_PEDAL: float = 0.
    RIGHT_PEDAL: float = 0.
    MIDDLE_PEDAL: float = 0.
    # Buttons - Bunch of unknowns so we add manually
    LS: bool = False
    RS: bool = False
    LB1: bool = False
    LB2: bool = False
    LB3: bool = False
    RB1: bool = False
    RB2: bool = False
    RB3: bool = False

    @classmethod
    def from_msg(cls, msg):
        return cls(
            msg.axes[:4],
            **dict(zip(['RS', 'LS'], msg.buttons[4:6])),
            **dict(zip(['RB1', 'LB1'], msg.buttons[6:8])),
            **dict(zip(['RB2', 'LB2'], msg.buttons[19:21])),
            **dict(zip(['RB3', 'LB3'], msg.buttons[21:23])),
        )

@dataclass
class LogitechG29(Joystick):
    ## Axes
    WHEEL: float = 0.
    LEFT_PEDAL: float = 0.
    RIGHT_PEDAL: float = 0.
    MIDDLE_PEDAL: float = 0.
    DPAD_X: float = 0.
    DPAD_Y: float = 0.
    ## Buttons
    CROSS: bool = False
    SQUARE: bool = False
    CIRCLE: bool = False
    TRIANGLE: bool = False
    SHIFT_UP: bool = False
    SHIFT_DOWN: bool = False
    R2: bool = False
    L2: bool = False
    SHARE: bool = False
    OPTION: bool = False
    R3: bool = False
    L3: bool = False
    STICK1: bool = False
    STICK2: bool = False
    STICK3: bool = False
    STICK4: bool = False
    STICK5: bool = False
    STICK6: bool = False
    STICK7: bool = False
    PLUS: bool = False
    MINUS: bool = False
    RING_CW: bool = False
    RING_CCW: bool = False
    ENTER: bool = False
    HOME: bool = False


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

        self._previous = None
        self._unused_btns = None

        init_method = getattr(self, f'init_{self.joy_kind}')
        init_method()

        self._btns = {}
        if self.joy_btns:
            for pair in self.joy_btns.split(','):
                assert ':' in pair, f'Invalid pair mapping: {pair}'
                btn, name = pair.split(':')
                if btn in self._unused_btns:
                    self._btns[btn] = self.create_client(Empty, name)
                    while not self._btns[btn].wait_for_service(timeout_sec=1.0):
                        logger.info(f'Service "{name}" not available, keep waiting...')
                    logger.debug(f'Button {btn} bound to service "{name}"')

    @rx.Timer(0.1)
    def loop(self):
        self.actuation.send_control(self._steering, self._velocity)

    @rx.Subscriber(Joy, joy_top)
    def joy_cb(self, msg):
        ## Interpret Message
        read_method = getattr(self, f'read_{self.joy_kind}')
        joy = read_method(msg)

        ## Button Services
        for btn, cli in self._btns.items():
            if self.release_event(joy, btn):
                self.get_logger().warn(f'Button Press Event: {btn}')
                cli.call_async(Empty.Request())

        ## Update history
        self._previous = joy

    def release_event(self, joy, name):
        return (False if self._previous is None else
                getattr(self._previous, name) and not getattr(joy, name))

    def init_xbox(self):
        self._previous = Xbox360Controller()
        self._unused_btns = set(Xbox360Controller.BUTTONS())
        self._unused_btns -= set('LB B X Y'.split())

    def read_xbox(self, msg):
        joy = Xbox360Controller.from_msg(msg)

        ## Velocity: Right Trigger (RT) + Left Trigger (LT)
        if joy.LT:
            self._velocity = - (1 - joy.LT)/2 * self.MAX_VELOCITY
        else:
            self._velocity = (1 - joy.RT)/2 * self.MAX_VELOCITY

        ## Steering: Left Stick
        self._steering = joy.LSX * self.MAX_STEERING

        ## Gear: Right Bumper (B) + Left Bumper (X)
        # at release
        if self.release_event(joy, 'B'):
            self.actuation.enable_highgear()
        if self.release_event(joy, 'X'):
            self.actuation.disable_highgear()

        ## Diff: Top Bumper (Y)
        # at release
        if self.release_event(joy, 'Y'):
            self.actuation.toggle_difflock()
        
        return joy

    def init_g29(self):
        self._previous = LogitechG29()
        self._unused_btns = set(LogitechG29.BUTTONS())
        self._unused_btns -= set(''.split())

    def read_g29(self, msg):
        joy = LogitechG29.from_msg(msg)

        ## Velocity: Right Pedal
        self._velocity = (1 - joy.RIGHT_PEDAL)/2 * self.MAX_VELOCITY
        if 0. < joy.LEFT_PEDAL < .75:
            self.get_logger().warn('Logitech G29: Unclear intent, LEFT_PEDAL pressed but not all the way for enabling reverse direction.')
            self._velocity = 0.
        elif .75 <= joy.LEFT_PEDAL:
            self._velocity *= -1

        ## Steering: Wheel
        self._steering = joy.WHEEL * self.MAX_STEERING

        ## Gear: SHIFT_UP + SHIFT_DOWN
        # at release
        if self.release_event(joy, 'SHIFT_UP'):
            self.actuation.enable_highgear()
        if self.release_event(joy, 'SHIFT_DOWN'):
            self.actuation.disable_highgear()

        return joy

if __name__ == '__main__':
    joy_consumer.main()


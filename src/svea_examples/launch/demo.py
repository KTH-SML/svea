#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this
from typing import Literal

@launch_this
def main(
    name: Literal['svea'] | Literal['teleop'],
    joy_kind: str = 'xbox',
    drycalls: bool = False,
    use_fmq: bool = True,
    use_joy: bool = False,
    start_joy: bool | Literal[...] = ...,
):
    bl = BetterLaunch()

    if start_joy is ...:
        start_joy = name == 'teleop'

    if name == 'svea':

        # Start SVEA system
        bl.include("svea_core", "svea.xml",
                   is_sim=False, use_lidar=False,
                   # qgc_host="10.0.0.28"
                   use_mavproxy=False)

        # Start Joy->SVEA translator
        if use_joy:
            bl.node("svea_examples", "joy_consumer.py",
                    name="joy_consumer",
                    params=dict(joy_top="/joy",
                                joy_kind=joy_kind,
                                joy_btns=','.join([
                                    "START:/qod",
                                    "BACK:/load_status",
                                    "DPADU:/load_on",
                                    "DPADD:/load_off",
                                ] if joy_kind == 'xbox' else [
                                    "ENTER:/qod",
                                    "SHARE:/load_status",
                                    "PLUS:/load_on",
                                    "MINUS:/load_off",
                                ] if joy_kind == 'g29' else [])))
        if not use_joy:
            bl.node("svea_examples", "twist_consumer.py",
                    name="twist_consumer",
                    params=dict(twist_top="fmq/remote_control",
                                twist_type="geometry_msgs/msg/TwistStamped"))

        # Start Ericsson API caller
        bl.node("svea_examples", "demo.py", 
                name="demo",
                params=dict(DRYCALLS=drycalls))

        if use_fmq:
            if use_joy:
                bl.node("ros_fmq_bridge", "bridge_node",
                        name="ros_fmq_bridge",
                        params=dict(subscribeTopics="/joy,",
                                    publishTopics="/load/status"))
            if not use_joy:
                bl.node("ros_fmq_bridge", "bridge_node",
                        name="ros_fmq_bridge",
                        params=dict(subscribeTopics="fmq/remote_control",
                                    publishTopics="/load/status"))

    if name == 'teleop':

        if use_fmq:
            bl.node("ros_fmq_bridge", "bridge_node",
                    name="ros_fmq_bridge",
                    params=dict(publishTopics="/joy",
                                subscribeTopics="/load/status"))

    if start_joy:
        # Start Joystick reader
        bl.node("joy", "joy_node", name="joy_node")


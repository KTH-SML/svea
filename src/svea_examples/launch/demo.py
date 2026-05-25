#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main(name: str):
    bl = BetterLaunch()

    if name == 'svea':
        bl.include("svea_core", "svea.xml",
                   is_sim=False, use_lidar=False,
                   # qgc_host="10.0.0.28"
                   use_mavproxy=False,
                   )
        bl.node("svea_examples", "joy_consumer.py",
                name="joy_consumer",
                params=dict(joy_top="/joy",
                            joy_btns=','.join([
                                "START:/qod",
                                "BACK:/load",
                            ])))
        bl.node("svea_examples", "demo.py", name="demo")
        bl.node("ros_fmq_bridge", "bridge_node",
                name="ros_fmq_bridge",
                params=dict(subscribeTopics="/joy"))

    if name == 'teleop':
        bl.node("joy", "joy_node", name="joy_node")
        bl.node("ros_fmq_bridge", "bridge_node",
                name="ros_fmq_bridge",
                params=dict(publishTopics="/joy"))

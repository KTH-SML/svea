#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main(name: str):
    bl = BetterLaunch()

    if name == 'svea':
        # bl.include("svea_core", "svea.xml")
        bl.node("svea_examples", "joy_consumer.py",
                name="joy_consumer",
                params=dict(joy_top="/joy"))
        bl.node("ros_fmq_bridge", "bridge_node",
                name="ros_fmq_bridge",
                params=dict(subscribeTopics="/joy"))

    if name == 'teleop':
        bl.node("joy", "joy_node", name="joy_node", params=dict(device_name="xbox"))
        bl.node("ros_fmq_bridge", "bridge_node",
                name="ros_fmq_bridge",
                params=dict(publishTopics="/joy"))



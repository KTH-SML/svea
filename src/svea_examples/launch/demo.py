#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main(name: str, joy_kind: str = 'xbox', drycalls: bool = False):
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
        bl.node("svea_examples", "demo.py", 
                name="demo",
                params=dict(DRYCALLS=drycalls))
        bl.node("ros_fmq_bridge", "bridge_node",
                name="ros_fmq_bridge",
                params=dict(subscribeTopics="/joy",
                            publishTopics="/load/status"))

    if name == 'teleop':
        bl.node("joy", "joy_node", name="joy_node")
        bl.node("ros_fmq_bridge", "bridge_node",
                name="ros_fmq_bridge",
                params=dict(publishTopics="/joy",
                            subscribeTopics="/load/status"))


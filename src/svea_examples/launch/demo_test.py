#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main():
    bl = BetterLaunch()

    bl.node("joy", "joy_node", name="joy_node")

    bl.node("svea_examples", "joy_consumer.py",
            name="joy_consumer",
            params=dict(joy_top="/joy",
                        joy_btns=','.join([
                            "START:/load",
                        ])))

    bl.node("svea_examples", "demo.py", name="demo")


#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main(
    name: str = 'self',
    use_sim_time: bool = False,
):
    bl = BetterLaunch()

    ROBOT = bl.read_robot_description("svea_core", "urdf/svea.urdf.xacro",
                                      xacro_args=["namespace:=$(var name)/"])

    with bl.group(ns=name):

        bl.node("robot_state_publisher", "robot_state_publisher",
                robot_description=ROBOT,
                use_sim_time=use_sim_time)

#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this
from better_launch.convenience import read_robot_description

@launch_this(screen_log_level='warning')
def main(
    name: str = 'self',
    use_sim_time: bool = False,
):
    bl = BetterLaunch()

    # TODO: This doens't work with multiple SVEAs, because the robot_description parameter is not namespaced. Need to fix this.
    return

    ROBOT = read_robot_description("svea_core", "svea.urdf.xacro",
                                   xacro_args=[f"namespace:={name}/"])

    with bl.group(name):

        bl.node("robot_state_publisher", "robot_state_publisher",
                name="urdf_visualization",
                params=dict(robot_description=ROBOT,
                            use_sim_time=use_sim_time))

#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main(
    name: str = 'self',
    map_pkg: str = 'svea_core',
    map_name: str = 'sml',
    initial_pose_x: float = 0.0,
    initial_pose_y: float = 0.0,
    initial_pose_a: float = 0.0,
    # Frames
    map_frame: str = 'map',
    odom_frame: str = '{name}/odom',
    base_frame: str = '{name}/base_link',
):
    bl = BetterLaunch()

    odom_frame = odom_frame.format(name=name)
    base_frame = base_frame.format(name=name)

    try:
        OBSTACLE_MAP = bl.load_params(map_pkg, f"{map_name}.obstacles.yaml", qualifier='**')
    except (ValueError, FileNotFoundError):
        OBSTACLE_MAP = {}

    with bl.group(name):

        # Start SVEA simulation
        bl.node("svea_core", "sim_svea.py",
                name="sim_svea",
                params=dict(initial_pose_x=initial_pose_x,
                            initial_pose_y=initial_pose_y,
                            initial_pose_a=initial_pose_a,
                            map_frame=map_frame,
                            odom_frame=odom_frame,
                            base_frame=base_frame))

        if OBSTACLE_MAP:
            # Start simulated LiDAR
            bl.node("svea_core", "sim_lidar.py",
                    name="sim_lidar",
                    params=OBSTACLE_MAP | dict(
                        laser_frame=f"{name}/laser",
                    ))

#!/usr/bin/env python3
import ast
import yaml
from better_launch import BetterLaunch, launch_this


def load_obstacles(bl, map_pkg, map_name):
    """Read <map_name>.obstacles.yaml and return the obstacle list.

    Returns [] if the file does not exist. ROS 2 parameters cannot hold
    nested arrays, so sim_lidar.py expects a string that it runs through
    ast.literal_eval() (see prepare_obstacles).
    """
    try:
        path = bl.find(map_pkg, f"{map_name}.obstacles.yaml")
    except Exception:
        return []

    with open(path) as f:
        data = yaml.safe_load(f) or {}

    if isinstance(data, dict):
        for value in data.values():
            if isinstance(value, dict) and 'ros__parameters' in value:
                data = value
                break
        if 'ros__parameters' in data:
            data = data['ros__parameters']
        data = data.get('obstacles', [])

    if isinstance(data, str):
        data = ast.literal_eval(data) if data.strip() else []

    return data or []


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

    OBSTACLES = load_obstacles(bl, map_pkg, map_name)

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

        if OBSTACLES:
            # Start simulated LiDAR
            bl.node("svea_core", "sim_lidar.py",
                    name="sim_lidar",
                    params=dict(laser_frame=f"{name}/laser",
                                odometry_topic="odometry/local",
                                obstacles=str(OBSTACLES)))
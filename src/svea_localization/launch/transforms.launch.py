#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

def toargs(**kwds):
    args = []
    for k, v in kwds.items():
        args.append(f"--{k.replace('_', '-')}")
        args.append(str(v))
    return args

@launch_this
def main(
    name: str = 'svea',
    use_gps: bool = True,
    use_lidar: bool = True,
    map_frame: str = 'map',
    odom_frame: str = 'odom',
    base_frame: str = 'base_link',
):
    bl = BetterLaunch()

    bl.node("tf2_ros", "static_transform_publisher",
            name="broadcaster_odom_map",
            cmd_args=toargs(x=0, y=0, z=0, yaw=0, pitch=0, roll=0,
                            frame_id=map_frame, child_frame_id=odom_frame))

    bl.node("tf2_ros", "static_transform_publisher",
            name="broadcaster_base_link_odom",
            cmd_args=toargs(x=0, y=0, z=0, yaw=0, pitch=0, roll=0,
                            frame_id=odom_frame, child_frame_id=base_frame))

    bl.node("tf2_ros", "static_transform_publisher",
            name="broadcaster_imu",
            cmd_args=toargs(x=0.10, y=-0.047, z=0.17, yaw=0, pitch=0, roll=0,
                            frame_id=base_frame, child_frame_id=f"{name}/imu"))

    if use_gps:
        bl.node("tf2_ros", "static_transform_publisher",
                name="broadcaster_gps",
                cmd_args=toargs(x=0.04, y=0.103, z=0.140, yaw=0, pitch=0, roll=0,
                                frame_id=base_frame, child_frame_id=f"{name}/gps"))

    if use_lidar:
        bl.node("tf2_ros", "static_transform_publisher",
                name="broadcaster_lidar",
                cmd_args=toargs(x=0.385, y=0, z=0.15, yaw=0, pitch=0, roll=0,
                                frame_id=base_frame, child_frame_id=f"{name}/laser"))

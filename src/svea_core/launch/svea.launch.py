#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main(
    name: str = 'self',
    is_sim: bool = True,
    is_indoor: bool = True,
    initial_pose_x: float = 0.0,
    initial_pose_y: float = 0.0,
    initial_pose_a: float = 0.0, # Yaw Angle
    ## Localization Settings
    use_localization: bool = True,
    ## Map
    use_map: bool = True,
    map_pkg: str = 'svea_core',
    map_name: str = 'sml',
    ## Low-Level Interface Settings
    lli_serial_device: str = '/dev/serial/by-id/usb-SVEA_PX4_AUTOPILOT_0-if00',
    lli_baud_rate: int = 921600,
    ## LiDAR Settings
    use_lidar: bool = True,
    ## RTK-GPS Settings
    use_rtk: bool = True,
    rtk_device: str = '/dev/ttyACM1',
    rtk_baud: int = 115200,
    rtk_username: str = '',
    rtk_password: str = '',
    ## External Communication
    use_zenoh: bool = False,
    ## Foxglove
    use_foxglove: bool = False,
    use_urdf: bool = False,
):
    bl = BetterLaunch()

    if use_localization:

        bl.include("svea_localization", "localization.launch.py",
                   name=name,
                   is_sim=is_sim,
                   is_indoor=is_indoor,
                   initial_pose_x=initial_pose_x,
                   initial_pose_y=initial_pose_y,
                   initial_pose_a=initial_pose_a,
                   use_map=use_map,
                   map_pkg=map_pkg,
                   map_name=map_name,
                   use_lidar=use_lidar,
                   use_rtk=use_rtk,
                   rtk_device=rtk_device,
                   rtk_baud=rtk_baud,
                   rtk_username=rtk_username,
                   rtk_password=rtk_password)

    if is_sim:

        bl.include("svea_core", "simulation.launch.py",
                   name=name,
                   map_pkg=map_pkg,
                   map_name=map_name,
                   initial_pose_x=initial_pose_x,
                   initial_pose_y=initial_pose_y,
                   initial_pose_a=initial_pose_a)
    
    else:

        ## Start Low-Level Interface (LLI)
        # with bl.group(name):
        bl.include("svea_core", "lli.xml",
                    name=name,
                    serial_device=lli_serial_device,
                    baud_rate=lli_baud_rate)
            

        # bl.node("svea_core", "lidar_timer.py",
        #         name="lidar_timer",
        #         params=dict(target_frame=f"{name}/base_link",
        #                     source_frame=f"{name}/odom",
        #                     from_topic=f"{name}/scan",
        #                     to_topic=f"{name}/scan/filtered"))

    if use_zenoh:

        ZENOH_CONFIG = bl.find("svea_core", "params/zenoh.json5")

        bl.process(f"zenoh-bridge-ros2dds -c {ZENOH_CONFIG}",
                   name="zenoh_bridge")

    if use_foxglove:
        bl.include("foxglove_bridge", "foxglove_bridge_launch.xml",
                   port=8765)

    if use_urdf:
        bl.include("svea_core", "visualization.launch.py",
                   name=name)


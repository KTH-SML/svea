#!/bin/bash python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main(
    name: str = 'self',
    is_sim: bool = True,
    is_indoor: bool = True,
    initial_pose_x: float = 0.0,
    initial_pose_y: float = 0.0,
    initial_pose_a: float = 0.0, # Yaw Angle
    map_pkg: str = 'svea_core',
    map_name: str = 'sml',
    use_urdf: bool = None,
    ## Low-Level Interface Settings
    lli_serial_device: str = '/dev/serial/by-id/usb-SVEA_PX4_AUTOPILOT_0-if00',
    lli_baud_rate: int = 921600,
    # mavproxy_qgc: str = 'host.docker.internal',
    ## Localization Settings
    use_localization: bool = True,
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
):
    bl = BetterLaunch()

    if use_urdf is None: use_urdf = is_sim

    if use_localization:

        bl.include("svea_localization", "localization.launch.py",
                   name=name,
                   is_sim=is_sim,
                   is_indoor=is_indoor,
                   initial_pose_x=initial_pose_x,
                   initial_pose_y=initial_pose_y,
                   initial_pose_a=initial_pose_a,
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

        with bl.group("mavros"):
            bl.node("mavros", "mavros_node",
                    cmd_args=["--ros-args",
                              "--log-level", "warn",
                              "--log-level", "mavros.sys:=fatal",
                              "--log-level", "mavros.cmd:=error",
                              "-r", "imu/diff_pressure:=_unused/imu/diff_pressure",
                              "-r", "imu/mag:=_unused/imu/mag",
                              "-r", "imu/static_pressure:=_unused/imu/static_pressure",
                              "-r", "imu/temperature_baro:=_unused/imu/temperature_baro",
                              "-r", "imu/temperature_imu:=_unused/imu/temperature_imu",
                              "-r", "manual_control/control:=_unused/manual_control/control",
                              "-r", "rc/out:=_unused/rc/out",
                              "-r", "rc/override:=_unused/rc/override",
                              "-r", "statustext/recv:=_unused/statustext/recv",
                              "-r", "statustext/send:=_unused/statustext/send",
                              "-r", "status_event:=_unused/status_event",
                              "-r", "sys_status:=_unused/sys_status",
                              "-r", "extended_state:=_unused/extended_state",
                              "-r", "estimator_status:=_unused/estimator_status",
                              "-r", "time_reference:=_unused/time_reference",
                              "-r", "timesync_status:=_unused/timesync_status"],
                    params=bl.load_params("svea_core", "mavros_pluginlists.yaml") | dict(
                        fcu_url=f"serial://{lli_serial_device}:{lli_baud_rate}",
                        # qgc_host="..."
                    ))
            
        bl.node("svea_core", "px4_uorb_tunnel.py",
                name="px4_uorb_tunnel",
                params=dict(mavlink_topic="/mavros/tunnel/out"))

        ## LLI Fixers
        
        bl.node("svea_core", "encoder_filter.py",
                name="encoder_filter")
        
        bl.node("svea_core", "imu_bias_remover.py",
                name="imu_bias_remover")

        bl.node("svea_core", "lidar_timer.py",
                name="lidar_timer",
                params=dict(target_frame=f"{name}/base_link",
                            source_frame=f"{name}/odom",
                            from_topic=f"{name}/scan",
                            to_topic=f"{name}/scan/filtered"))

    if use_urdf:
        bl.include("svea_core", "visualization.launch.py",
                   name=name)
        
    if use_zenoh:

        ZENOH_CONFIG = bl.find("svea_core", "params/zenoh.json5")

        bl.process(f"zenoh-bridge-ros2dds -c {ZENOH_CONFIG}",
                   name="zenoh_bridge")

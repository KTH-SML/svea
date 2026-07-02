#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main(
    name: str = 'self',
    is_sim: bool = True,
    is_indoor: bool = True,
    initial_pose_x: float = 0.0,
    initial_pose_y: float = 0.0,
    initial_pose_a: float = 0.0,
    map_pkg: str = 'svea_core',
    map_name: str = 'sml',
    # Coordinate Frames
    map_frame: str = 'map',
    odom_frame: str = '{name}/odom',
    base_frame: str = '{name}/base_link',
    # LiDAR Settings
    use_lidar: bool = True,
    lidar_ip: str = '192.168.0.10',
    # RTK-GPS Settings
    use_rtk: bool = True,
    rtk_device: str = '/dev/ttyACM1',
    rtk_baud: int = 115200,
    rtk_username: str = '',
    rtk_password: str = '',
    # Datum Settings
    use_datum: bool = False,
    datum_service: str = '/datum',
    datum_file: str = '',
    datum_data: str = '[]',
):
    bl = BetterLaunch()

    # Format the coordinate frames with the robot name
    map_frame = map_frame.format(name=name)
    odom_frame = odom_frame.format(name=name)
    base_frame = base_frame.format(name=name)

    with bl.group(name):

        # Static Transforms
        bl.include("svea_localization", "transforms.launch.py",
                   name=name,
                   use_gps=True,
                   use_lidar=use_lidar,
                   map_frame=map_frame,
                   odom_frame=odom_frame,
                   base_frame=base_frame)

    if is_sim:
        # Currently no localization is needed in simulation, as the simulator
        # provides perfect odometry and pose information. However, this section
        # can be expanded in the future to include simulated sensors and
        # localization algorithms if desired.
        pass

    else:

        USE_SIM_TIME = False

        # Load default parameters
        LOCAL_EKF_PARAMS = bl.load_params("svea_localization", "robot_localization/local_ekf.yaml")
        GLOBAL_EKF_PARAMS = bl.load_params("svea_localization", "robot_localization/global_ekf.yaml")
        AMCL_PARAMS = bl.load_params("svea_localization", "nav2_amcl/amcl.yaml")

        bl.node("robot_localization", "ekf_node",
                name="ekf_local",
                params=LOCAL_EKF_PARAMS | dict(
                    map_frame=map_frame,
                    odom_frame=odom_frame,
                    base_link_frame=base_frame,
                    world_frame=odom_frame,
                    imu0="/lli/filtered/imu",
                    twist0="/lli/filtered/encoders"
                ),
                remap={"/odometry/filtered": f"{name}/odometry/local"})

        if is_indoor:

            if use_lidar:
                with bl.group(name):
                    bl.include("svea_localization", "lidar.launch.py",
                               lidar_ip=lidar_ip,
                               lidar_frame=f"{name}/laser")

            bl.node("nav2_amcl", "amcl",
                    name="amcl",
                    params=AMCL_PARAMS | dict(
                        use_sim_time=USE_SIM_TIME,
                        yaml_filename=bl.find(map_pkg, f"{map_name}.yaml"),
                        scan_topic=f"{name}/scan/filtered",
                        initial_pose_x=initial_pose_x,
                        initial_pose_y=initial_pose_y,
                        initial_pose_a=initial_pose_a,
                        base_frame_id=base_frame,
                        odom_frame_id=odom_frame,
                        map_frame_id=map_frame,
                        map_ztopic="/map",
                    ))
            bl.node("nav2_lifecycle_manager", "lifecycle_manager",
                    name="lifecycle_manager_amcl",
                    params=dict(use_sim_time=USE_SIM_TIME,
                                autostart=True,
                                node_names=["amcl"]))

        else:

            if use_rtk:
                bl.include("svea_localization", "rtk.launch.py",
                           device=rtk_device,
                           baud=rtk_baud,
                           username=rtk_username,
                           password=rtk_password)
                
            # Start NavSat Transform Node
            bl.node("robot_localization", "navsat_transform_node",
                    name="navsat_transform_node",
                    params=dict(publish_filtered_gps=True,
                                wait_for_datum=use_datum,
                                delay=2.0,
                                magnetic_declination_radians=0.0,
                                yaw_offset=initial_pose_a,
                                zero_altitude=True,
                                broadcast_cartesian_transform_as_parent_frame=True,
                                broadcast_cartesian_transform=True),
                    ## TODO
                    # remap= {'imu/data': '/imu/data',
                    #         'gps/fix': '/gps/fix',
                    #         'odometry/filtered': '/odometry/filtered/global'}
                    )

            # Start Set Datum Node
            if use_datum:
                bl.node("svea_localization", "set_datum_node.py",
                        name="set_datum_node",
                        output="screen",
                        params=dict(datum_service=datum_service,
                                    service_timeout=60.0,
                                    datum_file=datum_file,
                                    datum_data=datum_data))

            bl.node("robot_localization", "ekf_node",
                    name="ekf_global",
                    params=GLOBAL_EKF_PARAMS | dict(
                        map_frame=map_frame,
                        odom_frame=odom_frame,
                        base_link_frame=base_frame,
                        world_frame=map_frame,
                        imu0="/lli/filtered/imu",
                        twist0="/lli/filtered/encoders",
                        odom0=f"{name}/odometry/gps"
                    ),
                    remap={"/odometry/filtered": f"{name}/odometry/global"})

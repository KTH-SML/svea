#!/usr/bin/env python3
"""
SLAM bringup for SVEA.

Place at: src/svea_localization/launch/slam.launch.py

Usage (real car):
    bl svea_localization slam.launch.py
    bl svea_localization slam.launch.py name:=self use_foxglove:=True

IMPORTANT: this launch file deliberately does NOT start map_server or AMCL.
While mapping, slam_toolbox owns the  map -> <name>/odom  transform. If AMCL
or the static map->odom broadcaster in transforms.launch.py is also running,
you get two publishers on the same TF edge and the map will jump around.
"""

from better_launch import BetterLaunch, launch_this


def toargs(**kwds):
    args = []
    for k, v in kwds.items():
        args.append(f"--{k.replace('_', '-')}")
        args.append(str(v))
    return args


def find_params(bl, pkg, filename):
    """Locate a params file regardless of whether setup.py flattens subdirs.

    svea_localization/setup.py may or may not preserve params/<subdir>/ structure,
    so try the likely locations instead of hardcoding one.
    """
    candidates = (
        filename,
        f"params/{filename}",
        f"params/slam_toolbox/{filename}",
        f"params/robot_localization/{filename}",
    )
    for candidate in candidates:
        try:
            return bl.find(pkg, candidate)
        except (ValueError, FileNotFoundError):
            continue
    raise FileNotFoundError(
        f"Could not find {filename} in {pkg}. Tried: {candidates}. "
        f"Check the install space: ls install/{pkg}/share/{pkg}/params/"
    )


@launch_this
def main(
    name: str = "self",
    ## Low-Level Interface (mavros / PX4)
    lli_serial_device: str = "/dev/serial/by-id/usb-SVEA_PX4_AUTOPILOT_0-if00",
    lli_baud_rate: int = 921600,
    ## LiDAR
    use_lidar: bool = True,
    lidar_ip: str = "192.168.0.10",
    # Encoder Settings
    use_two_encoders: bool = True,
    ## Local EKF (odom -> base_link)
    use_ekf: bool = True,
    ekf_params: str = "local_ekf.yaml",
    ## SLAM
    slam_mode: str = "async",              # "async" or "sync"
    slam_params: str = "slam_sync.yaml",
    ## Tools
    use_foxglove: bool = True,
    foxglove_port: int = 8765,
):
    bl = BetterLaunch()

    # Frames — same convention as localization.launch.py
    map_frame = "map"
    odom_frame = f"{name}/odom"
    base_frame = f"{name}/base_link"
    laser_frame = f"{name}/laser"
    imu_frame = f"{name}/imu"

    # ------------------------------------------------------------------
    # Low-Level Interface
    # ------------------------------------------------------------------
    bl.include("svea_core", "lli.xml",
               name=name,
               serial_device=lli_serial_device,
               baud_rate=lli_baud_rate)

    with bl.group(name):

        # --------------------------------------------------------------
        # Static transforms
        # --------------------------------------------------------------
        # Only the sensor mounts. No map->odom (slam_toolbox publishes it) and
        # no odom->base_link (ekf_local publishes it).
        bl.node("tf2_ros", "static_transform_publisher",
                name="broadcaster_imu",
                cmd_args=toargs(x=0.10, y=-0.047, z=0.17,
                                yaw=0, pitch=0, roll=0,
                                frame_id=base_frame, child_frame_id=imu_frame))

        bl.node("tf2_ros", "static_transform_publisher",
                name="broadcaster_lidar",
                cmd_args=toargs(x=0.385, y=0.0, z=0.15,
                                yaw=0, pitch=0, roll=0,
                                frame_id=base_frame, child_frame_id=laser_frame))

        # --------------------------------------------------------------
        # LiDAR
        # --------------------------------------------------------------
        if use_lidar:
            bl.include("svea_localization", "lidar.launch.py",
                       lidar_ip=lidar_ip,
                       lidar_frame=laser_frame)

        # --------------------------------------------------------------
        # Local EKF:  <name>/odom -> <name>/base_link
        # --------------------------------------------------------------
        if use_ekf:

            if use_two_encoders:
                ekf_input = {"twist0": f"/{name}/mavros/wheel_odometry/odom"}
            else:
                ekf_input = {"twist0": f"/{name}/wheel_odometry/twist/filtered"}
                bl.node("svea_localization", "single_encoder_twist_filter.py",
                            name="wheel_twist",
                            params={"base_frame":       base_frame,
                                    "distance_topic":   f"/{name}/mavros/wheel_odometry/distance",
                                    "twist_topic":      ekf_input["twist0"],
                                    "imu_topic":        f"/{name}/mavros/imu/data_raw",
                                    "rc_topic":         f"/{name}/mavros/rc/in",
                                    "control_topic":    f"/{name}/mavros/manual_control/send"})
            
            EKF_PARAMS_FILE = find_params(bl, "svea_localization", ekf_params)
            bl.node("robot_localization", "ekf_node",
                    name="ekf_local",
                    cmd_args=["--ros-args", "--params-file", EKF_PARAMS_FILE],
                    params={"map_frame": map_frame,
                            "odom_frame": odom_frame,
                            "base_link_frame": base_frame,
                            "world_frame": odom_frame,
                            "twist0": ekf_input["twist0"]},
                    remaps={"odometry/filtered": "odometry/local"})

        # --------------------------------------------------------------
        # SLAM:  map -> <name>/odom
        # --------------------------------------------------------------
        slam_exec = ("async_slam_toolbox_node" if slam_mode == "async"
                     else "sync_slam_toolbox_node")

        SLAM_PARAMS = find_params(bl, "svea_localization", slam_params)

        bl.node("slam_toolbox", slam_exec,
                name="slam_toolbox",
                cmd_args=["--ros-args", "--params-file", SLAM_PARAMS],
                params={"map_frame": map_frame,
                        "odom_frame": odom_frame,
                        "base_frame": base_frame,
                        "scan_topic": f"/{name}/scan",
                        "mode": "mapping",
                        "use_sim_time": False},
                remaps={"map": "/map", "map_metadata": "/map_metadata"})

    # ------------------------------------------------------------------
    # Foxglove
    # ------------------------------------------------------------------
    if use_foxglove:
        bl.include("foxglove_bridge", "foxglove_bridge_launch.xml",
                   port=foxglove_port)

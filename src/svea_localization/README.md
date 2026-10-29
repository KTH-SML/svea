# svea_localization

Localization, sensor drivers and mapping launches for SVEA on ROS 2 Jazzy.
For installation, use the [Docker workflow](../../docs/setup/docker.md) or
[native build instructions](../../docs/development/native_build.md).

## Launch files

| File | Purpose |
| --- | --- |
| `localization.launch.py` | Map and transforms; hardware adds local EKF and indoor AMCL or outdoor RTK/navsat/global EKF. Simulation uses simulator odometry. |
| `transforms.launch.py` | Static transforms for vehicle and sensor frames. |
| `slam.launch.py` | MAVROS, wheel/IMU odometry, LiDAR and SLAM Toolbox. |
| `lidar.launch.py` | Hokuyo driver. See [LiDAR setup](../../docs/setup/lidar_setup.md). |
| `rtk.launch.py` | RTK receiver and NTRIP client. See [outdoor localization status](../../docs/development/outdoor_localization.md). |
| `_localization_debugger.launch.py` | Unimplemented; raises an assertion. |

Sensor launches are installed alongside the other launch files. Run them by
filename, for example `ros2 launch svea_localization lidar.launch.py`.

## Localization defaults

| Argument | Default |
| --- | --- |
| `name` | `self` |
| `is_sim`, `is_indoor` | `True` |
| `map_frame` | `map` |
| `odom_frame` | `{name}/odom` |
| `base_frame` | `{name}/base_link` |
| `map_pkg`, `map_name` | `svea_core`, `sml` |
| `lidar_ip` | `192.168.0.10` |
| `rtk_device`, `rtk_baud` | `/dev/ttyACM1`, 115200 |

On hardware, the local EKF reads MAVROS IMU and wheel odometry and publishes
`/{name}/odometry/local`. Indoor AMCL provides map alignment. With one encoder,
`use_two_encoders=False` enables the wheel-twist filter.

The standalone localization launch needs a running PX4 connection on hardware.
`svea_core/svea.launch.py` combines both. See
[ROS connection](../../docs/setup/ros_bringup.md) and
[Better Launch boolean arguments](../../docs/reference/rosonic.md).

## Parameter files

- `params/nav2_amcl/amcl.yaml`
- `params/robot_localization/local_ekf.yaml`
- `params/robot_localization/global_ekf.yaml`
- `params/slam_toolbox/slam_sync.yaml`
- `params/slam_toolbox/localization.yaml`

Rebuild after adding or changing installed configuration, then source the
workspace. See [MoCap localization](../../docs/mocap_localization.md) for
DISCOWER tracking and controller integration.

The outdoor pipeline has unresolved remappings and datum setup; its status
and blockers are documented in the [outdoor guide](../../docs/development/outdoor_localization.md).

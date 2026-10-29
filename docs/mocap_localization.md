# MoCap Localization (Qualisys)

SVEA uses [DISCOWER/motion_capture_system](https://github.com/DISCOWER/motion_capture_system).
The base Docker image includes its `mocap_qualisys` driver and the `svea_mocap`
launch wrapper.

## Before starting

In QTM, configure and calibrate the capture workspace, define the vehicle's
rigid body and enable real-time 6DOF output. Note the server address and
rigid-body name.

Run the container on a Linux computer with access to the QTM network, using
`util/run` for host networking. QTM streaming also needs incoming UDP traffic.
For TCP streaming, the DISCOWER launch accepts `udp_port:=-1`.

## Start tracking

The SVEA wrapper uses server `10.0.0.10`, base port 22222 and fixed frame `mocap`:

```bash
ros2 launch svea_mocap mocap.launch.py
```

It also publishes the static transform `mocap → map`. Set `map_x` and `map_y`
to the map origin's measured position in metres, and `map_a` to its yaw in
radians. All three default to zero; use the measured alignment for your map.

### Another QTM server

Start the DISCOWER driver directly instead of the wrapper:

```bash
ros2 launch mocap_qualisys qualisys.launch.py   server_address:=10.0.0.10 server_base_port:=22222   fixed_frame_id:=mocap publish_tf:=true
```

Replace the example server address. This starts tracking; configure the
`mocap → map` transform separately in your launch. The SVEA wrapper only exposes
map alignment and frame arguments, so server settings belong in this direct launch.

## Check tracking

For a QTM rigid body named `self`, without an added driver namespace:

| Topic | Type |
| --- | --- |
| `/self/pose` | `geometry_msgs/msg/PoseStamped` |
| `/self/odom` | `nav_msgs/msg/Odometry` |
| `/self/velocity` | `geometry_msgs/msg/TwistStamped` |

Replace `self` with your rigid-body name:

```bash
ros2 topic echo /self/odom --once
ros2 run tf2_ros tf2_echo mocap self
ros2 run tf2_ros tf2_echo mocap map
```

Check that position and orientation follow the vehicle and agree with the map.
With TF enabled, the driver publishes `mocap → self`; odometry uses fixed frame
`mocap` and child frame `self/base_link`. Align the tracked body's origin and
axes with the vehicle base, or supply the measured transform between them.

## Feed a controller

Set these `LocalizationInterface` parameters for your vehicle:

| Parameter | Value |
| --- | --- |
| `localization/odom_top` | `/self/odom` |
| `localization/map_frame` | `map` |
| `localization/base_frame` | The calibrated vehicle base frame |

Provide the TF chain between these frames. Choose one source for each TF
relationship to avoid conflicting transforms. Tracking alone does not start
PX4 communication or a controller; see [ROS connection](setup/ros_bringup.md).

Topic and frame conventions were checked against DISCOWER
[commit 37e3093](https://github.com/DISCOWER/motion_capture_system/tree/37e3093c251207726996e965a50e9fa17145508e).

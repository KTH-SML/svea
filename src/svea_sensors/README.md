# SVEA sensors

## Contents

1. [Overview](#overview)
    1. [License](#license)
    2. [Supported Sensors](#supported-sensors)
2. [Installation](#installation)
    1. [Building from Source](#building-from-source)
    2. [ZED mini camera installation](#zed-mini-camera-installation)
    3. [Realsense T265 installation](#realsense-t265-installation)
    4. [Hokuyo UST-10LX installation](#hokuyo-ust-10lx-installation)
3. [Usage](#usage)
4. [Launch files](#launch-files)
5. [Config files](#config-files)
6. [Nodes](#nodes)
    1. [bnoo055_i2c_node](#bnoo055_i2c_node)
    2. [odom_to_map](#odom_to_map)
    3. [actuation_to_twist](#actuation_to_twist)
    4. [wheel_encoder_reader](#wheel_encoder_reader)

## Overview

Contains settings and software for all default sensors of the SVEA vehicles.

### License

Svea sensors is Copyright (c) 2019, Tobias Bolin and released under the
[MIT license](svea_sensors/LICENSE.md)

[tf2_ros](src/noetic_tf2_ros) is Copyright (c) 2008, Willow Garage, Inc. and
released under the [BSD 3-Clause license](svea_sensors/src/imu_bno055/LICENSE.txt).

[imu_bno055](src/imu_bno055) is Copyright (c) 2019, Dheera Venkatraman and
released under the [BSD 3-Clause license](svea_sensors/src/imu_bno055/LICENSE.txt).

### Supported Sensors

The package includes launch files, transforms, dependencies etc. for the
following sensors:

-   **`ZED mini`**
    Stereo camera for visual odometry. Publish rate 15-100 Hz depending on
    resolution. Note that the transform for the ZED is set in
    [urdf/zedm.urdf](urdf/zedm.urdf), and not in
    [luanch/transforms.launch](luanch/transforms.launch).

-   **`Intel Realsense T265`**
    Stereo camera for visual odometry. Publish rate 200 Hz (visual odometry at
    about 30 Hz integrated with IMU data).

-   **`IMU`**
    BNO055, 9 degrees of freedom IMU. Publish rate ca 100 Hz. (Can also give
    temperature)

-   **`Hokuyo UST-10LX`**
    2D lidar with a publish rate of 40 Hz. Connected through ethernet.

-   **`Wheel encoders`**
    The wheel encoders are a prototype and not available on most SVEAs. The
    software required for using a pair of encoders is however included in this
    package.

-   **`ZED-F9P Satellite Navigation and RTK`** Multi-band receiver providing centimeter accuracy for outdoors navigation when connected to the [SWEPOS Network RTK](https://www.lantmateriet.se/en/geodata/gps-geodesi-och-swepos/swepos/swepos-services/network-rtk/connection-to-the-service/) service. We use the following module vendor `MIKROE-3881`. Publish rate 1 Hz.

## Installation

The package should be downloaded and used as a part of the `svea` package.

The _Hokuyo UST-10LX_, _Realsense T265_, _ZED mini_, and _ZED-F9P_ requires special
installation steps.

**Note:** After installing the package you will have to enable additional
hardware permisions inorder to connect to the sensors. Do this by running

    roscd svea_sensors
    sudo bash grant_hardware_permisions.sh

and then restart the computer.

### Building from Source

#### Building

To build from source, clone the latest version from this repository into your
catkin workspace and compile the package using

```
cd catkin_workspace/src
git clone https://github.com/KTH-SML/svea_research
cd ../
rosdep install --from-paths . --ignore-src
catkin build
```

### ZED mini camera installation

-   Install the ZED SDK according to the
    [ZED SDK install instructions](https://www.stereolabs.com/docs/installation/jetson/#download-and-install-the-zed-sdk).

-   Install the [ZED ROS wrapper](https://github.com/stereolabs/zed-ros-wrapper#build-the-program).

### Realsense T265 installation

These instructions are for installing the Intel Realsense T265 camera on a
Nvidia Jetson. The default package that can be downloaded with `apt-get` have
some bugs on the Jetson computers. Instead the packages will have to be
installed from jetsonhacks, and the ROS package will have to be cloned from
github. Start by adding the jetsonhacks repository with:

```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
```

Note that `bionic` might have to be replaced if the ubuntu version switches
from 18.

Then install the required realsense packages with

```
sudo apt-get install apt-utils -y
sudo apt-get install librealsense2-utils librealsense2-dev -y
```

The ROS realsense repo can then be cloned (and checked out to the correct
tag) into your `[workspace_name]/src` directory:

```
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout 2.2.16
```

In Jetpack opencv is installed in `/usr/include;/usr/include/opencv4`
instead of `/usr/include/opencv` where ROS (more specifically `cv_bridge`)
will look for it. Fix this by adding a symlink to the correct dirctory with

```
sudo ln -s /usr/include/opencv4 /usr/include/opencv
```

Run `rosdep` to make sure that all dependencies are installed.

```
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

And finally build and source the workspace.

```
catkin build
source devel/setup.bash
```

### Hokuyo UST-10LX installation

The ethernet interface to which the lidar is connected has to be set to a manual IP, within the same range as the lidar. For example: if the lidar's IP is `192.168.3.11` the ethernet interface on the Jetson should be set to something like

```
IP: 192.168.3.10
Netmask: 255.255.255.0
```

## Usage

Start the self localization with

```
roslaunch svea_sensors localize.launch rviz:=true
```

Rviz should started and display the position of the vehicle. In addition,
odometry for the vehicle in the _odom_ frame of reference will be published
on `odometry/filtered`, and the same information translated to the _map_
frame will be published on `odometry/corrected`.

## Launch files

-   **localize.launch:** Starts the sensors, static transforms, the
    localization node, and the pose transformation node.

    General arguments

    -   **`xavier`** If the transform should be as they are on SVEAs with a
        Xavier AGX. Default: `false`.

    -   **`use_camera`** Use camera odometry, in contrast to bag file. Default:
        `true`.

    -   **`camera`** Name of the camera to be used (either `zed` or `rs`).
        Default: `rs`.

    Localization arguments

    -   **`map_file`** Full path to the map file. If empty, don't start map
        server. Default: empty.

    -   **`initial_pose_x`** Initial position guess on the x axis in meters `0.0`.

    -   **`initial_pose_y`** Initial position guess on the y axis in meters
        `map_name`. Default: `0.0`.

    -   **`initial_pose_a`** Initial guess for the yaw in radians. Default: `0.0`.

    Pose translation arguments

    -   **`map_frame`** Name of the corrected map frame. Default: `map`.

    -   **`wait_for_transform`** If false will use the last available
        transform directly. If true will wait for another transform to be
        available and interpolate. Setting `wait_for_transform` to true will
        increase the delay and is therefore not recommended when controling
        physical robots. Default: `false`.

    -   **`publish_odometry`** If a translated odometry message should be
        published in addition to the state. Default: `true`.

    -   **`publish_pose`** If a translated pose message should be published in
        addition to the state. Default: `true`.

-   **rviz.launch:** Launch `rviz` with a configuration suitable for localization.

-   **slam.launch:** Start the sensors and a `slam_toolbox` node to do
    simultaneous localization and mapping (SLAM).

    General arguments

    -   **`xavier`** If the transform should be as they are on SVEAs with a
        Xavier AGX. Default: `false`.

    -   **`camera`** Name of the camera to be used (either `zed` or `rs`).
        Default: `rs`.

    -   **`slam_delay`** The launch of the SLAM node will be delayed with this
        many seconds to give the sensors time to start . Default: `5`.

-   **slam_and_bag.launch:** Launches all things that `slam.launch` starts. In
    addition also launches a `rosbag record` node that records relevant sensor
    data to do SLAM off line.

    General arguments

    -   **`xavier`** If the transform should be as they are on SVEAs with a
        Xavier AGX. Default: `false`.

    -   **`camera`** Name of the camera to be used (either `zed` or `rs`).
        Default: `rs`.

    -   **`slam_delay`** The launch of the SLAM node will be delayed with this
        many seconds to give the sensors time to start . Default: `5`.

    -   **`output_dir`** Tha bags are stored in this directory. **Will fail if
        the directory does not exist!** Default: home directory.

-   **slam_from_bag.launch:** Do SLAM from a bag file recrded with
    `slam_and_bag.launch`.

    General arguments

    -   **`file_name`** Name of the file in `file_path` that should be used
        for. Required.

    -   **`file_path`** The folder that the bag file is in. Default:
        `/home/$(env USER)/bagfiles/`.

    -   **`xavier`** If the transform should be as they are on SVEAs with a
        Xavier AGX. Default: `false`.

    -   **`use_rs`** If the RS T265 should be used instead of the ZED camera.
        Default: `false`.

    -   **`rviz`** Launch rviz together with the localization. Default: `false`.

    -   **`slam_delay`** The launch of the SLAM node will be delayed with this
        many seconds to give the sensors time to start . Default: `5`.

-   **wheel_odometry.launch:** Start the wheel encoder node.

    -   **`direction_topic`** Topic that will be used for infering direction.
        Default: `actuation_twist`.

    -   **`encoder_frame`** Transform frame id of the encoders. Default:
        `base_link`.

    -   **`encoder_topic`** Topic that the encoder node should read from.
        Default: `lli/encoder`.

    -   **`start_actuation_reader`** Will start an actuation reader node for
        direction if true. Default: `true`.

    -   **`start_serial`** Will start arosserial node for direction if true.
        Default: `true`.

    -   **`vehicle_name`** Name of the vehicle. Default: Empty string.

-   **transforms.launch:** Publishes static transforms for the SVEA

    -   **`xavier`** Use transforms for the AXG Xavier version of the SVEA if
        `True` . Default: `false`.

-   **rs_odometry.launch:** Start odometry based on the RS T265

    -   **`lidar_ip`** IP address to the Hokuyo LIDAR. Default: `192.168.3.11`.

    -   **`xavier`** Use transforms for the AXG Xavier version of the SVEA if
        `True`. Default: `false`.

-   **zed_odometry.launch:** Start odometry based on the ZED Mini

    -   **`lidar_ip`** IP address to the Hokuyo LIDAR. Default: `192.168.3.11`.

    -   **`node_name`** Name for the ZED camera node. Default: `zed_node`.

    -   **`publish_urdf`** Wheter URDF based transforms for the ZED should be
        published. Default: `true`.

    -   **`xavier`** Use transforms for the AXG Xavier version of the SVEA if
        `True`. Default: `false`.

-   **rtk.launch:** Start publishing `gps/fix` messages and launch the **ntrip_client** which
    communicates with the services providing _RTCM_ correction data to the receiver via serial port. The overall setup is described in the following diagram

    ```mermaid
        flowchart LR
            A[Swepos NTRIP Server]
            AA[NTRIP Client Node]
            B[RTK Manager Node]
            C[ZED-F9P]
            T1{{/gps/fix}}
            T2{{/gps/heading_accuracy}}
            T3{{/gps/heading_motion}}
            T4{{/gps/heading_vehicle}}
            T5{{/gps/magnetic_declination}}
            T6{{/gps/speed}}
            T7{{/gps/speed_accuracy}}

            AA -->|1. Send NMEA| A
            A -->|2. Receive RTCM| AA
            B -->|publish /ntrip_client/nmea|AA
            AA -->|subscribe /ntrip_client/rtcm | B
            B -->|write RTCM over serial|C
            C -->|read NMEA, NAV-PVT, and NAV-COV over serial|B
            B -->|pub|T1
            B -->|pub|T2
            B -->|pub|T3
            B -->|pub|T4
            B -->|pub|T5
            B -->|pub|T6
            B -->|pub|T7
    ```

    General arguments

    -   **`device`** Port to the USB device of the ZED-F9P receiver. Default: `/dev/ttyACM0`.
    -   **`baud`** Baud rate for USB device. Default: `250000`.
    -   **`gps_frame`** frame_id of all GPS messages. Default: `gps`.
    -   **`dynamic_model`** sets dynamic model for ZED-F9P RTK receiver. Default `portable`. Allowed values are: `portable`, `stationary`, `pedestrian`,`automotive`,`sea`, `airborne_1g`, `airborne_2g`, `airborne_4g`,`wrist_watch`, and `bike`.

    NTRIP Client arguments (Explained in details [here](http://wiki.ros.org/ntrip_client))

    -   **`host`** Hostname of the _NTRIP_ server that the client will receive corrections from. Default: `ntrt-swepos.lm.se`. Alternatively, you can use [rtk2go.com](http://rtk2go.com:2101/SNIP::STATUS).
    -   **`port`** Port that the _NTRIP_ server is listening on. Default: `80`.
    -   **`authenticate`** whether to authenticate with _NTRIP_ server when connecting. Default `true`.
    -   **`mountpoint`**
    -   **`username`** username to authenticate with _NTRIP_ server.
    -   **`password`** password to authenticate with _NTRIP_ server. For SWEPOS Network RTK Credentials, open [this document](https://kth.sharepoint.com/:w:/s/ITRL/EQpnEBUVJVdMrDuXIj8IMBUBuqc_rFoeRelxt1d4YaZ71Q?e=Q4i3nz) (only for KTH team members).

## Config files

AMCL config [params/amcl](params/amcl)

-   **loaclize.yaml** Localization configurations for AMCL.

Robot localization config [params/robot_localization](params/robot_localization)

-   **ekf_template.yaml** Template file with descriptions of parameters.

-   **rs_ekf.yaml** Odometry sensorfusion configurations for the RS T265.

-   **zed_ekf.yaml** Odometry sensorfusion configurations for the ZED Mini.

Rviz config [params/rviz](params/rviz)

-   **slam_toolbox.rviz** Rviz layout to be used with the SLAM toolbox.

Slam toolbox [params/slam_toolbox](params/slam_toolbox)

-   **slam_sync.yaml** Default parameters for SLAM

-   **localize.yaml** Parameters for using SLAM toolbox localization (experimental).

ZED [params/zed](params/zed)

-   **common.yaml** Common parametes for ZED and Zed Mini cameras

-   **zed.yaml** Parameters for ZED camera

-   **zedm.yaml** Parameters for ZED Mini camera

URDF [urdf/](urdf/)

-   **zed.urdf** URDF setup for ZED camera

-   **zedm.urdf** URDF setup (including transform to SVEA base_link) for ZED mini camera

## Nodes

### bnoo055_i2c_node

This is a ROS node for the BNO055 IMU that communicates via I2C and without
any dependencies besides libi2c-dev. It does **not** require RTIMULib,
RTIMULib2, RTIMULib3 or whatever the latest sequel is. It is specifically
targeted at using a BNO055 with NVIDIA boards such as the TX1, TX2, and
Xavier, or any other board that has native I2C.

The BNO055 supports I2C and UART communication. This driver supports I2C
only. If you are looking for a UART driver, see
[this driver](https://github.com/mdrwiega/bosch_imu_driver)
by [mdrwiega](https://github.com/mdrwiega) instead.

#### Published Topics

-   **`data`** ([sensor_msgs/Imu])

    Acceleration, angular velocity and orientation.
    The orientation is calculated by integrating the angular velocity
    onboard the BNO055. The BNNO055 is not configured to include magnetometer
    readings due to the inherent problems with magnetometers. The orintation
    is therefore relative to the orientation when the BNO055 was started.

-   **`raw`** ([sensor_msgs/Imu])

    Un-corrected acceleration and angular velocity.
    The acceleration is not corrected by the BNO055 self calibration, hwever
    it appears to still be heavily filtered. The angular velocity is the same
    as published on the _data_ topic.

-   **`mag`** ([sensor_msgs/MagneticField])

    The current magnetic field reading.

-   **`temp`** ([sensor_msgs/Temperature])

    The temperature measured by the BNO055.

#### Parameters

-   **`rate`** (int, default: 400)

    Rate that the BNO055 will be pooled at.
    Data will only be published if any of the values has changed compared to
    the last published values. The BNO055 supports a maximum rate of 100 Hz.
    A higher rate decreases the delay of that reading.

-   **`device`** (string, default: "/dev/i2c-1")

    Path to the i2c device. Default is /dev/i2c-1. Use i2cdetect in the
    i2c-tools package to find out which bus your IMU is on. For a TX2 this is
    usually `"/dev/i2c-2"` and for the Xavier AGX it is usually `"/dev/i2c-8"`.

-   **`address`** (int, default: 0x28)

    i2c address of the IMU.

-   **`frame_id`** (string, default: "imu")

    The frame id used in published messages.

### odom_to_map

A node that translate odometry messages published in an odom frame onto a map
based on a transform from e. g. AMCL.

#### Subscribed Topics

-   **`odometry/filtered`** ([nav_msgs/Odometry])

    Topic for the odometry messages that will be translated.

#### Published Topics

-   **`state`** ([svea_msgs/VehicleState])

    Kinematic bicycle state of the vehicle in the map frame.

-   **`odometry/corrected`** ([nav_msgs/Odometry])

    Odometry of the vehicle in the map frame.

-   **`pose`** ([geometry_msgs/PoseWithCovarianceStamped])

    Pose of the vehicle in the map frame.

#### Parameters

-   **`map_frame`** (stricng, default: "map")

    Frame of reference that the odometry messages should be translated to.

-   **`wait_for_transform`** (bool, default: false)

    If set to _false_ the last received transform sill be used to translate
    an odometry message. If _true_ the node will wait for the next transform.
    Waiting for the transform will cause a delay, but could allow higher
    accuracy since an interpolation between two transformes can be used for
    the translation.

-   **`publish_odometry`** (bool, default: true)

    Set to false to stop publishing odometry messages.

-   **`publish_pose`** (bool, default: true)

    Set to false to stop publishing pose messages.

### actuation_to_twist

Approximates the current velocity based on the actuated throttle.
The calculation accounts for the current gear and acceleration characteristics of the vehicle. The velocity is published at a fixed rate, using zero order hold interpolation. That is, the last received message is assumed to be in effect until the next message is received.

#### Subscribed Topics

-   **`lli/encoder`** ([svea_msgs/lli_encoder])

    Raw encoder data that is used to calculate the velocity.

#### Published Topics

-   **`actuation_twist`** ([geometry_msgs/TwistWithCovarianceStamped])

    Approximate velocity of the vehicle based on the actuations.

#### Parameters

-   **`ctrl_message_topic`** (string, Default: "lli/ctrl_actuated")

    Topic where the actuation messages are published.

-   **`twist_message_topic`** (string, Default: "actuation_twist")

    Topic where this node will publish twist messages with the calculated vlocity.

-   **`frame_id`** (string, Default: "base_link")

    Frame of reference of the vehicle.

-   **`max_speed_0`** (float, Default: 1.7)

    Max speed of the vehicle in low gear.

-   **`max_speed_1`** (float, Default: 3.6)

    Max speed of the vehicle in high gear.

-   **`max_steering_angle`** (float, Default: 40 degrees)

    Max steering angle of the vehicle.

-   **`linear_covariance`** (float, Default: 0.1)

    Linear covariance that will be published in the twist messages.

-   **`angular_covariance`** (float, Default: 0.1)

    Angular covariance that will be published in the twist messages.

-   **`rate`** (float, Default: 50)

    Publishing rate of the twist messages.

-   **`tau0`** (float, Default: 0.1)

    Time constant of velociy in low gear.

-   **`tau1`** (float, Default: 0.4)

    Time constant of velocity in high gear.

### wheel_encoder_reader

Reads raw encoder values and calculates the vehicle's velocity based on a
given wheel radius. Can optionaly use another source of velocity to infere
the direction of the velocity.

#### Subscribed Topics

-   **`lli/encoder`** ([svea_msgs/lli_encoder])

    Topic where raw encoder data is published.

-   **`actuation_twist`** ([geometry_msgs/TwistWithCovarianceStamped])

    Topic that will be used to infere the direction of the encoders.

#### Published Topics

-   **`wheel_encoder_twist`** ([geometry_msgs/TwistWithCovarianceStamped])

    Velocity of the vehicle as given by the encoders.

#### Parameters

-   **`encoder_frame`** (string, Default: "base_link")

    Frame of reference of the encoders.

-   **`encoder_topic`** (string, Default: "lli/encoder")

    Topic that publishes the encoder messages read by the node.

-   **`direction_topic`** (string, Default: "actuation_twist")

    Topic used to infere the direction of the encoders.
    If empty or no messages are published on the topic, the directin will be
    assumed to be forward.

-   **`axle_track`** (float, Default: 199.0)

    Axle track i. e. distance between the wheels of the vehicle in mm.

-   **`wheel_radius`** (float, Default: 60.0)

    Radius of the wheels in mm.

-   **`linear_covariance`** (float, Default: 0.2)

    Linear coavriance that will be included in the twist message published by
    the node.

-   **`angular_covariance`** (float, Deault: 0.4)

    Angular coavriance that will be included in the twist message published
    by the node.

[geometry_msgs/PoseWithCovarianceStamped]: http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
[geometry_msgs/TwistWithCovarianceStamped]: http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html
[nav_msgs/Odometry]: http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html
[sensor_msgs/Imu]: http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
[sensor_msgs/Temperature]: http://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html
[sensor_msgs/MagneticField]: http://docs.ros.org/en/api/sensor_msgs/html/msg/MagneticField.html

### rtk_manager

Reads `NAV_PVT`, `NAV_COV`, and `NMEA` messages over serial port and publishes `gps` topics. It also publishes `NMEA` messages to the `ntrip_client` node and subscribes to `RTCM` messages from the `ntrip_client` which it writes to the ZED-F9P receiver.

#### Subscribed Topics

from mavros_msgs.msg import RTCM

-   **`/ntrip_client/rtcm`** ([mavros_msgs/RTCM])

    Topic where the NTRIP client publishes the correction messages from the NTRIP server.

#### Published Topics

-   **`/gps/fix`** ([sensor_msgs/NavSatFix])

    GPS fix message

-   **`/gps/speed`** ([std_msgs/Float64])

    Ground speend in 2-D in [m/s]

-   **`/gps/speed_accuracy`** ([std_msgs/Float64])

    Estimate of ground speed accuracy in [m/s]

-   **`/gps/heading_motion`** ([std_msgs/Float64])

    Heading of 2-D motion in [deg]

-   **`/gps/heading_vehicle`** ([std_msgs/Float64])

    Heading of vehicle in 2-D in [deg]

-   **`/gps/heading_accuracy`** ([std_msgs/Float64])

    Combined heading accuracy of vehicle and motion heading in [deg]

-   **`/gps/magnetic_declination`** ([std_msgs/Float64])

    Magnetic declination in [deg]

-   **`/gps/magnetic_declinantion_accuracy`** ([std_msgs/Float64])

    Accuracy of magnetic declination in [deg]

-   **`/ntrip_client/nmea`** ([nmea_msgs/Sentence])

    NMEA message used by the NTRIP Client to annouce location to a virtual NTRIP server.

#### Parameters

-   **`device`** (string, Default: "ttyACM0")

    Serial device port to the ZED-F9P receiver connected via USB.

-   **`baud`** (int, Default: "45000")

    Rate of serial port

-   **`gps_frame`** (string, Default: "gps")

    Name of frame_id for the gps topics published from node.

-   **`dynamic_model`** (string, Default: "portable")

    Dynamic model to use by the RTK receiver. Allowed values are "portable", "stationary", "pedestrian", "automotive", "sea", "airborne_1g", "airborne_2g", "airborne_4g", "wrist_watch", and "bike"

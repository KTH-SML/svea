# SVEA sensors

## Overview
Contains settings and software for all default sensors of the SVEA vehicles.

### License

Svea sensors is Copyright (c) 2019, Tobias Bolin and released under the [MIT license](svea_sensors/LICENSE.md)

[tf2_ros](src/noetic_tf2_ros) is Copyright (c) 2008, Willow Garage, Inc. and released under the [BSD 3-Clause license](svea_sensors/src/imu_bno055/LICENSE.txt).

[imu_bno055](src/imu_bno055) is Copyright (c) 2019, Dheera Venkatraman and released under the [BSD 3-Clause license](svea_sensors/src/imu_bno055/LICENSE.txt).


**Author: Tobias Bolin<br />
Affiliation: [KTH Royal Institute of Technology](https://www.kth.se/)<br />
Maintainer: Tobias Bolin, tbolin@kth.se**

### Supported Sensors
The package includes launch files, transforms, dependencies etc. for the following sensors:

* **`ZED mini`**
Stereo camera for visual odometry. Publish rate 15-100 Hz depending on resolution.
Note that the transform for the ZED is set in [urdf/zedm.urdf](urdf/zedm.urdf), and not in [luanch/transforms.launch](luanch/transforms.launch).

* **`Intel Realsense T265`**
Stereo camera for visual odometry. Publish rate 200 Hz (visual odometry at about 30 Hz integrated with IMU data).

* **`IMU`**
BNO055, 9 degrees of freedom IMU. Publish rate ca 100 Hz.
(can also give temperature) 

* **`Hokuyo UST-10LX`**
2D lidar with a publish rate of 40 Hz. Connected through ethernet. 

* **`Wheel encoders`**
The wheel encoders are a prototype and not available on most SVEAs. The software required for using a pair of encoders is however included in this package. 


## Installation
The package should be downloaded and used as a part of the `svea` package.

The *Hokuyo UST-10LX*, *Realsense T265*, and *ZED mini* requires special installation steps.

**Note:** After installing the package you will have to enable additional hardware permisions inorder to connect to the sensors. Do this by running 

    roscd svea_sensors
    sudo bash grant_hardware_permisions.sh

and then restar the computer.

### Building from Source

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/KTH-SML/svea_research
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin build


### ZED mini camera installation

* Install the ZED SDK according to the 
[ZED SDK install instructions](https://www.stereolabs.com/docs/installation/jetson/#download-and-install-the-zed-sdk).

* Install the [ZED ROS wrapper](https://github.com/stereolabs/zed-ros-wrapper#build-the-program)

### Realsense T265 installation

These instructions are for installing the Intel Realsense T265 camera on a Nvidia Jetson. The default package that can be downloaded with `apt-get` have some bugs on the Jetson computers. 
Instead the packages will have to be installed from jetsonhacks, and the ROS package will have to be cloned from github.
Start by adding the jetsonhacks repository with:

```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
```

Note that `bionic` might have to be replaced if the ubuntu version switches from 18.

Then install the required realsense packages with

```sudo apt-get install apt-utils -y
sudo apt-get install librealsense2-utils librealsense2-dev -y
```

The ROS realsense repo can then be cloned (and checked out to the correct tag) into your `[workspace_name]/src` directory:

```
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout 2.2.16
```

In Jetpack opencv is installed in `/usr/include;/usr/include/opencv4`
instead of `/usr/include/opencv` where ROS (more specifically `cv_bridge`) will look for it.
Fix this by adding a symlink to the correct dirctory with
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

    IP: 192.168.3.10
    Netmask: 255.255.255.0

## Usage

Start the self localization with 

    roslaunch svea_sensors localize.launch rvis:=true

Rviz should started and display the position of the vehicle.
In addition, odometry for the vehicle in the *odom* frame of reference will be published on `odometry/filtered`,
and the same information translated to the *map* frame will be published on `odometry/corrected`.

## Launch files
* **localize.launch:** Starts the sensors, static transforms, the localization node, and the pose transformation node.

    General arguments
     
    - **`xavier`** If the transform should be as they are on SVEAs with a Xavier AGX. Default: `false`.

    - **`use_rs`** If the RS T265 should be used instead of the ZED camera. Default: `true`.
    
    - **`rviz`** Launch rviz together with the localization. Default: `false`.

    - **`start_sensors`** Start the sensors. Turn of if you are running from a bag file. Default: `true`.
    
    - **`start_serial`** Start a rosserial node. Default: `false`.
    
    Localization arguments
    
    - **`map_name`** Name of the file in /maps/ that should be used for localization. Default: `itrl.yaml`.
    
    - **`file_name`** Full path to the map file. Will override `map_name`. Default: `$(find svea_sensors)/maps/$(arg file_name)`.

    - **`initial_pose_x`** Initial position guess on the x axis in meters `0.0`.
    
    - **`initial_pose_y`** Initial position guess on the y axis in meters `map_name`. Default: `0.0`.
    
    - **`initial_pose_a`** Initial guess for the yaw in radians. Default: `0.0`.
    
     Pose translation arguments

     - **`map_frame`** Name of the corrected map frame. Default: `map`.
     
     - **`wait_for_transform`** If false will use the last available transform directly. If true  will wait for another transform to be available and interpolate. Setting `wait_for_transform` to true will increase the delay and is therefore not recommended when controling physical robots. Default: `false`.
     
     - **`publish_odometry`** If a translated odometry message should be published in addition to the state. Default: `true`.
     
     - **`publish_pose`** If a translated pose message should be published in addition to the state. Default: `true`.
    
* **rviz.launch:** Launch `rviz` with a configuration suitable for localization.
     

* **slam.launch:** Start the sensors and a `slam_toolbox` node to do simultaneous localization and mapping (SLAM). 

     General arguments
     
    - **`xavier`** If the transform should be as they are on SVEAs with a Xavier AGX. Default: `false`.

    - **`use_rs`** If the RS T265 should be used instead of the ZED camera. Default: `false`.
    
    - **`rviz`** Launch rviz together with the localization. Default: `false`.
   
    - **`slam_delay`** The launch of the SLAM node will be delayed with this many seconds to give the sensors time to start . Default: `5`.

* **slam_and_bag.launch:** Launches all things that `slam.launch` starts. In addition also launches a `rosbag record` node that records relevant sensor data to do SLAM off line. Tha bags are stored in `/home/$(env USER)/bagfiles/`. **Will fail if the directory does not exist!**

    General arguments
     
    - **`xavier`** If the transform should be as they are on SVEAs with a Xavier AGX. Default: `false`.

    - **`use_rs`** If the RS T265 should be used instead of the ZED camera. Default: `false`.
    
    - **`rviz`** Launch rviz together with the localization. Default: `false`.
   
    - **`slam_delay`** The launch of the SLAM node will be delayed with this many seconds to give the sensors time to start . Default: `5`.
    
* **slam_from_bag.launch:** Do SLAM from a bag file recrded with `slam_and_bag.launch`.
    
    General arguments
    
    - **`file_name`** Name of the file in `file_path` that should be used for. Required.
     
    - **`file_path`** The folder that the bag file is in. Default: `/home/$(env USER)/bagfiles/`. 
    
    - **`xavier`** If the transform should be as they are on SVEAs with a Xavier AGX. Default: `false`.

    - **`use_rs`** If the RS T265 should be used instead of the ZED camera. Default: `false`.
    
    - **`rviz`** Launch rviz together with the localization. Default: `false`.
   
    - **`slam_delay`** The launch of the SLAM node will be delayed with this many seconds to give the sensors time to start . Default: `5`.

* **show_map.launch:** Start a `map_server` that publishes a map file so it can be viewed in `rviz`.
    
    General arguments
    
    - **`map_name`** Name of the file in /maps/ that should be used for localization. Default: `itrl.yaml`.
    
    - **`file_name`** Full path to the map file. Will override `map_name`. Default: `$(find svea_sensors)/maps/$(arg file_name)`.


* **wheel_odometry.launch:** Start the wheel encoder node.

    - **`direction_topic`** Topic that will be used for infering direction. Default: `actuation_twist`.

    - **`encoder_frame`** Transform frame id of the encoders. Default: `base_link`.

    - **`encoder_topic`** Topic that the encoder node should read from. Default: `lli/encoder`.

    - **`start_actuation_reader`** Will start an actuation reader node for direction if true. Default: `true`.

    - **`start_serial`** Will start arosserial node for direction if true. Default: `true`.

    - **`vehicle_name`** Name of the vehicle. Default: Empty string.


* **transforms.launch:** Publishes static transforms for the SVEA

    - **`xavier`** Use transforms for the AXG Xavier version of the SVEA if `True` . Default: `false`.


* **rs_odometry.launch:** Start odometry based on the RS T265

    - **`ip_address`** IP address to the Hokuyo LIDAR. Default: `192.168.3.11`.

    - **`start_serial`** Start a rosserial node for contact with the low level interface if `true` (reuired by the `actuation_to_twist` node). Default: `false`.

    - **`xavier`** Use transforms for the AXG Xavier version of the SVEA if `True`. Default: `false`.

* **zed_odometry.launch:** Start odometry based on the ZED Mini

    - **`ip_address`** IP address to the Hokuyo LIDAR. Default: `192.168.3.11`.

    - **`node_name`** Name for the ZED camera node. Default: `zed_node`.

    - **`publish_urdf`** Wheter URDF based transforms for the ZED should be published. Default: `true`.

    - **`xavier`** Use transforms for the AXG Xavier version of the SVEA if `True`. Default: `false`.

## Config files

AMCL config [params/amcl](params/amcl)

* **loaclize.yaml** Localization configurations for AMCL.

Robot localization config [params/robot_localization](params/robot_localization)

* **ekf_template.yaml** Template file with descriptions of parameters.

* **rs_ekf.yaml** Odometry sensorfusion configurations for the RS T265.

* **zed_ekf.yaml** Odometry sensorfusion configurations for the ZED Mini.

Rviz config [params/rviz](params/rviz)

* **slam_toolbox.rviz** Rviz layout to be used with the SLAM toolbox.

Slam toolbox [params/slam_toolbox](params/slam_toolbox)

* **slam_sync.yaml** Default parameters for SLAM

* **localize.yaml** Parameters for using SLAM toolbox localization (experimental).

ZED [params/zed](params/zed)

* **common.yaml** Common parametes for ZED and Zed Mini cameras

* **zed.yaml** Parameters for ZED camera

* **zedm.yaml** Parameters for ZED Mini camera 

URDF [urdf/](urdf/)

* **zed.urdf** URDF setup for ZED camera

* **zedm.urdf** URDF setup (including transform to SVEA base_link) for ZED mini camera


## Nodes

### bnoo055_i2c_node
This is a ROS node for the BNO055 IMU that communicates via I2C and without any dependencies besides libi2c-dev. It does **not** require RTIMULib, RTIMULib2, RTIMULib3 or whatever the latest sequel is. It is specifically targeted at using a BNO055 with NVIDIA boards such as the TX1, TX2, and Xavier, or any other board that has native I2C.

The BNO055 supports I2C and UART communication. This driver supports I2C only. If you are looking for a UART driver, see [this driver](https://github.com/mdrwiega/bosch_imu_driver) by [mdrwiega](https://github.com/mdrwiega) instead.

#### Published Topics

* **`data`** ([sensor_msgs/Imu])
    
    Acceleration, angular velocity and orientation. 
    The orientation is calculated by integrating the angular velocity
    onboard the BNO055. The BNNO055 is not configured to include magnetometer readings due to the inherent problems with magnetometers. The orintation is therefore relative to the orientation when the BNO055 was started.

* **`raw`** ([sensor_msgs/Imu])

    Un-corrected acceleration and angular velocity.
    The acceleration is not corrected by the BNO055 self calibration, hwever it appears to still be heavily filtered. The angular velocity is the same as published on the *data* topic.

* **`mag`** ([sensor_msgs/MagneticField])

    The current magnetic field reading.

* **`temp`** ([sensor_msgs/Temperature])

    The temperature measured by the BNO055.

#### Parameters

* **`rate`** (int, default: 400)

	Rate that the BNO055 will be pooled at.
    Data will only be published if any of the values has changed compared to the last published values. The BNO055 supports a maximum rate of 100 Hz. A higher rate decreases the delay of that reading.

* **`device`** (string, default: "/dev/i2c-1")
    
    Path to the i2c device. Default is /dev/i2c-1. Use i2cdetect in the i2c-tools package to find out which bus your IMU is on.
    For a TX2 this is usually `"/dev/i2c-2"` and for the Xavier AGX
    it is usually `"/dev/i2c-8"`.

* **`address`** (int, default: 0x28)

    i2c address of the IMU.

* **`frame_id`** (string, default: "imu")

    The frame id used in published messages.


### odom_to_map_node
A node that translate odometry messages published in an odom frame onto a map based on a transform from e. g. AMCL.

#### Subscribed Topics

* **`odometry/filtered`** ([nav_msgs/Odometry])

    Topic for the odometry messages that will be translated.


#### Published Topics

* **`state`** ([svea_msgs/VehicleState])

    Kinematic bicycle state of the vehicle in the map frame.

* **`odometry/corrected`** ([nav_msgs/Odometry])

    Odometry of the vehicle in the map frame.

* **`pose`** ([geometry_msgs/PoseWithCovarianceStamped])

    Pose of the vehicle in the map frame.


#### Parameters

* **`map_frame`** (stricng, default: "map")

    Frame of reference that the odometry messages should be translated to.

* **`wait_for_transform`** (bool, default: false)

    If set to *false* the last received transform sill be used to translate an odometry message. If *true* the node will wait for the next transform. Waiting for the transform will cause a delay, but could allow higher accuracy since an  interpolation between two transformes can be used for the translation.

* **`publish_odometry`** (bool, default: true)

    Set to false to stop publishing odometry messages.

* **`publish_pose`** (bool, default: true)

    Set to false to stop publishing pose messages.


### actuation_to_twist

Approximates the current velocity based on the actuated throttle.
The calculation accounts for the current gear and acceleration characteristics of the vehicle. The velocity is published at a fixed rate, using zero order hold interpolation. That is, the last received message is assumed to be in effect until the next message is received.

#### Subscribed Topics

* **`lli/encoder`** ([svea_msgs/lli_encoder])

    Raw encoder data that is used to calculate the velocity.

#### Published Topics

* **`actuation_twist`** ([geometry_msgs/TwistWithCovarianceStamped])

    Approximate velocity of the vehicle based on the actuations.


#### Parameters

* **`ctrl_message_topic`** (string, Default: "lli/ctrl_actuated")

    Topic where the actuation messages are published.

* **`twist_message_topic`** (string, Default: "actuation_twist")

    Topic where this node will publish twist messages with the calculated vlocity.

* **`frame_id`** (string, Default: "base_link")

    Frame of reference of the vehicle.

* **`max_speed_0`** (float, Default: 1.7)

    Max speed of the vehicle in low gear.

* **`max_speed_1`** (float, Default: 3.6)

    Max speed of the vehicle in high gear.

* **`max_steering_angle`** (float, Default: 40 degrees)

    Max steering angle of the vehicle.

* **`linear_covariance`** (float, Default: 0.1)

    Linear covariance that will be published in the twist messages.

* **`angular_covariance`** (float, Default: 0.1)

    Angular covariance that will be published in the twist messages.

* **`rate`** (float, Default: 50)

    Publishing rate of the twist messages.

* **`tau0`** (float, Default: 0.1)

    Time constant of velociy in low gear.

* **`tau1`** (float, Default: 0.4)

    Time constant of velocity in high gear.



### wheel_encoder_reader
Reads raw encoder values and calculates the vehicle's velocity based on a given wheel radius. Can optionaly use another source of velocity to infere the direction of the velocity.

#### Subscribed Topics

* **`lli/encoder`** ([svea_msgs/lli_encoder])

    Topic where raw encoder data is published.


* **`actuation_twist`** ([geometry_msgs/TwistWithCovarianceStamped])

    Topic that will be used to infere the direction of the encoders.


#### Published Topics

* **`wheel_encoder_twist`** ([geometry_msgs/TwistWithCovarianceStamped])

    Velocity of the vehicle as given by the encoders.

#### Parameters

* **`encoder_frame`** (string, Default: "base_link")

    Frame of reference of the encoders.

* **`encoder_topic`** (string, Default: "lli/encoder")

    Topic that publishes the encoder messages read by the node.

* **`direction_topic`** (string, Default: "actuation_twist")

    Topic used to infere the direction of the encoders. 
    If empty or no messages are published on the topic, the directin will be assumed to be forward.

* **`axle_track`** (float, Default: 199.0)

    Axle track i. e. distance between the wheels of the vehicle in mm.

* **`wheel_radius`** (float, Default: 60.0)

    Radius of the wheels in mm.

* **`linear_covariance`** (float, Default: 0.2)

    Linear coavriance that will be included in the twist message published by the node.

* **`angular_covariance`** (float, Deault: 0.4)

    Angular coavriance that will be included in the twist message published by the node.



[geometry_msgs/PoseWithCovarianceStamped]: http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
[geometry_msgs/TwistWithCovarianceStamped]: http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html
[nav_msgs/Odometry]: http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html
[sensor_msgs/Imu]: http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
[sensor_msgs/Temperature]: http://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html
[sensor_msgs/MagneticField]: http://docs.ros.org/en/api/sensor_msgs/html/msg/MagneticField.html

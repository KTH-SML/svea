# SVEA sensor pakcage
Contains settings and software for all default sensors of the SVEA vehicles.
The realsense T265 camera and ZED camera requires special installation steps. 


## Installation
The package should be downloaded as a part of `svea_research`.

### Installing the ZED camera

### Installing the Realsense T265 camera
These instructions are for installing the Intel Realsense T265 camera. The default package that can be downloaded with `apt-get` have some bugs on the Jetson computers. 
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
catkin_make
source devel/setup.bash
```

### Installation from Github

The `svea_sensors` package should be downloaded and built as part of the 'svea_research' repository.

## Launch files
* **localize.launch:** Starts the sensors, static transforms, the localization node, and the pose transformation node.

    General arguments
     
    - **`xavier`** If the transform should be as they are on SVEAs with a Xavier AGX. Default: `false`.

    - **`use_rs`** If the RS T265 should be used instead of the ZED camera. Default: `false`.
    
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


## Sensors

### ZED cam
Stereo camera for visual odometry. Publish rate 15-100 Hz.

### Intel Realsense T265 
Alternative stereo camera for visual odometry. Publish rate 200 Hz (visual odometry at about 30 Hz integrated with IMU data).

### IMU
BNO055, 9 degrees of freedom IMU. Publish rate ca 100 Hz.
(can also give temperature) 

### Hokyo Lidar
Connected through ethernet. Requires a little bit of setup. Publish rate 40 Hz.

### Wheel encoders

## Published Topics
This list is incomplete, since especially the zed camera publishes a lot of topics.
See the individual packages for better descriptions. 

### /odometry/filtered
Fused output from the EKF.


 




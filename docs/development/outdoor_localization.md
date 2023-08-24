# Outdoor Localization for SVEA

## Content
1. [Overview](#overview)
2. [Usage](#usage)
3. [Launch files](#launch-files)
4. [Config files](#config-files)
5. [Nodes](#nodes)

## Overview

Description for all the scripts that are related to the outdoor localization stack.

## Usage
1. Go to [this document](https://kth.sharepoint.com/:w:/s/ITRL/EQpnEBUVJVdMrDuXIj8IMBUBuqc_rFoeRelxt1d4YaZ71Q?e=Q4i3nz) (only for KTH team members) and copy one of the usersname and password to the rtk.launch file
2. Make sure the RTK-GPS is connected, and the realsense camera and 4K Logitech camera with the usb-c connection are disconnected. Ensure the SVEA is connected to the remote controller, so that you can always stop it when needed.
3. Start the outdoor localization stack with  
```
roslaunch svea_examples outdoor_test.launch device:=<port_location_for_rtk_gps>
```

## Launch files
### rtk.launch
This launch file is for starting the RTK-GPS unit.

Run the following to startup:
```
roslaunch svea_sensors rtk.launch device:=<port_location>
```
-   The port is fixed to `/dev/GPS` for SVEA2.

If the RTK-GPS stratup successfully, the terminal should show 
```
Connected to http://nrtk-swepos.lm.se:80/MSM_GNSS
```
To check the covariance and the GPS reading, use 
```
rostopic echo /gps/fix
```
The GPS reading is accurate enough when the covariance has a magnitude of less than 1e-4.

-   Notes:
    -   The GPS unit does not work well with the 4K Logitech camera and the realsense camera due to interference.
    -   The GPS must be placed far from the building, trees or other huge obstacles in order to obtain a more accurate reading.
    -   The username and password can be found in [this document](https://kth.sharepoint.com/:w:/s/ITRL/EQpnEBUVJVdMrDuXIj8IMBUBuqc_rFoeRelxt1d4YaZ71Q?e=Q4i3nz) (only for KTH team members).

### navsat.launch
This launch file is fro starting the navsat_transform_node, which is used to transform the RTK-GPS location to usable data for EKF global.

-   Notes:
    -   **`yaw_offset`**: The IMU has a default mapping for cardinal directions, i.e. for our IMU, 0 -> North. The assumption used for the navsat_transform_node is 0 -> East. Thus, this paramter should be set to pi/2. Unless the IMU is changed, you don't need to adjust this parameter.

    -   **`broadcast_utm_transform_as_parent_frame`**: To add the UTM frame as a parent of map frame.
    -    **`broadcast_utm_transform`**: To add the UTM frame. This parameter has to be set to true in order to use `broadcast_utm_transform_as_parent_frame`.

### rs_odometry.launch
This launch file is for starting the IMU, statc transforms, actuation_to_twist, EKF local and global. 

### transforms.launch
This lauch file is for starting all the required static transforms for the SVEA. 

### localize.launch
This launch file is for starting the rs_odometry.launch, serial_node, wheel_encoder, map, rtk.launch, navsat.launch and odom_to_map. 

General parameters

-   **`use_wheel_encoders`**: Set to be true if you are using svea7. Default: ``. 
-   **`start_serial`**: Must be set to true for motor. Default: `false`.
-   **`is_indoors`**: Must be set to false. Default: `true`.
-   **`device`**: The port where RTK-GPS is. Default:`/dev/GPS`.

### outdoor_test.launch
Autonomous driving with outdoor localization stack.
-   Parameters
    -   **`resolution`**: The number of division for each path. Default: `10`.
    -   **`corners`**: This parameter contains all the GPS coordinates (List of list that contains the lat, long of each point) that are the target points for the SVEA. To collect the GPS location for the target points, simply start the rtk.launch file, and drive the SVEA to the target positin in the physical environment. Make sure when you collect the GPS location, the rtk has a low covariance (magnitude of 1e-4 or smaller).
    -   **`use_wheel_encoders`**: true if the script is running on SVEA7.
    -   **`initial_pose_x`**, **`initial_pose_y`**, **`initial_pose_a`**: initial pose of SVEA (x,y,theta)
    -   *Below are for your reference, in most cases, you do not need to change these parameters*
        -   **`waypoints_topic`**: The topic which the target points to form the path are published to. Default: `/outdoor_localization_waypoint`.
        -   **`location_topic`**: The topic that publishes the GPS location of the SVEA. Default: `/gps/filtered`.
        -   **`marker_topic`**: The topic that publishes the visualization markers (The position of the target points and the initial pose of SVEA) in rViz. Default: `/waypoints`.
        -   **`gps_odometry_topic`**: The topic that publishes the SVEA pose (x,y) in map frame. Default: `/odometry/filtered/global`.
-   Note:
    -   **ALWAYS UNPLUG THE REALSENSE CAMERA AND THE 4K LOGITECH CAMERA (THE ONE WITH THE USBC CONNECTION) IF YOU ARE USING THE RTK-GPS**
    -   It might take a few trials before the RTK-GPS connects to the service.

## Config files

### global_ekf.yaml
Sensor fusion for EKF global. This includes `/odometry/gps`, `/imu/data`, `/actuation_twist`, `/wheel_encoder_twist`.
-   Parameters
    -   **`odom0_pose_rejection_threshold`**: To rejct inaccurate RTK-GPS measurement. If the RTK-GPS measurement is too far away (larger than the threshold) from the current location, the RTK-GPS data will be ignored. If wheel encoder is used, this value can be smaller.

### rs_ekf.yaml
Sensor fusion for EKF loval. This includes `/imu/data`, `/actuation_twist`, `/wheel_encoder_twist`.
-   Notes:
    -   **`/rs/t265_camera/odom/sample`**: This parameter is used for indoor localization, or when the RTK-GPS is not used.

## Nodes

### relative_waypoints.py
This node takes in the target points (in GPS coordinate) and calculates the relative distance between the target points and the initial position of the SVEA, and publishes this points in map frame (x,y) to `/outdoor_localization_waypoint`.

### outdoor_test.py
A node that has a very similar functionalities as the pure_pursuit.py, but it subscribes to `/outdoor_localization_waypoint` and use these points to generate the trajectory for pure pursuit.

### plot_localization.py
This node is for plotting the SVEA's path recorded by the RTK-GPS and EKF. It is best used for reviewing a rosbag. (Foxglove studio is also useful)

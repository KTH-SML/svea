# Outdoor Localization for SVEA

## Contents

## Overview

Description for all the scripts that is related to the outdoor localization stack.

## Usage
Start the outdoor localization stack wtith  
```
roslaunch svea_sensors localize.launch
```

## Launch files
### rtk.launch
This part is only useful if you would like to run RTK-GPS independently. This launch file is already included in the `localize.launch`.

The RTK-GPS has to be plugged in and run the following to startup
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

### camera.launch

For starting the web camera

### get_aruco_utm.launch (change to static_utm_transform.launch)


### navsat.launch

### rs_odometry.launch

### transforms.launch

### localize.launch
Starts the sensors, static transforms, the localization node, and the pose transformation node.

General parameters

-   **`use_wheel_encoders`**: Set to be true if you are using svea7. Default: ``. 
-   **`start_serial`**: Must be set to true for motor. Default: ``.
-   **`is_indoors`**: Must be set to false. Default: ``.
-   **`device`**: The port where RTK-GPS is. Default:`/dev/GPS`.
-   **`yaw_offset`**: The offset that define the strating heading of SVEA. 0 when facing East. Default:`0`.

Parameters for sensor fusion with ArUco marker (To be continued)

-   **`use_web_camera`**: Set to true if you want to use the web camera on SVEA for aruco detection. Default: ``.
-   **`image`**: . Default: ``.
-   **`aruco_pose`**:. Default: ``.
-   **`aruco_dict`**: . Default: ``.
-   **`aruco_size`**: The width/height of the ArUco marker that you will use. Default: ``.
-   **`aruco_tf_name`**: . Default: ``.
-   **`device_id`**: The port for the video input (check with ```ls /dev```). Default: ``.
-   **`static_gps_fix_topic`**: . Default: ``.
-   **`odometry_gps_topic`**: . Default: ``.
-   **`aruco_pose_topic`**: . Default: ``.
-   **`aruco_id`**: The aruco id that you will use for detection. Default: ``.

### outdoor_autonomous_driving.launch

Autonomous driving with outdoor localization stack.
On top of the parameters mentioned above for localize.launch
-   **`generate_waypoints`**: Set to true if you want to run the SVEA autonomously with outdoor localization stack. Default: ``.
-   **`waypoints_topic`**: . Default: ``.
-   **`resolution`**: The number of division for each path. Default: ``.
-   **`location_topic`**: . Default: ``.
-   **`marker_topic`**: . Default: ``.
-   **`gps_odometry_topic`**: . Default: ``.

### pedestrian_demo.launch

Demo for Kista Mobility Day.
On top of the parameters mentioned above for localize.launch
-   **`pedestrian_demo`**: Set to true if you want to run together with the sidewalk_monility_demo. Default: ``.
-   **`testing_pedestrian_demo`**: . Default: ``.


## Yaml file

### global_ekf.yaml

### rs_ekf.yaml

### usb_web_camera.yaml

### web_camera.yaml

## Script

### relative_waypoints.py

### pure_pursuit_outdoor.py

### aruco_detect.py

### aruco_pose.py

### pedestrian_location.py

### publish_rsu_msg.py

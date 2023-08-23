## RTK-GPS Implementation
###   global_ekf.yaml: 
-   Changes:
    -   added **`odom0_pose_rejection_threshold`** and set to 1. If the RTK-GPS measurement is too far away (larger than the threshold) from the current location, the RTK-GPS data will be ignored. If wheel encoder is used, this value can be smaller.
    -   changed **`two_d_mode`** to true to operate the EKF in 2D
    -   changed **`imu0_differential`** to true as the imu always start from 0
    -   changed **`imu0_relative`** to false as the imu always start from 0
-   Performance:
    -   In worse case scenrio, the RTK-GPS drifts up to 40-50cm (even when the covariance for the RTK-GPS signal is small). In best case scenrio, the RTK-GPS can achieve an accuracy of 5-10cm. 
###   navsat.launch:
-   Changes:
    -   added **`broadcast_utm_transform_as_parent_frame`** and 
set to true to add the UTM frame as a parent of map frame.
    -   added **`broadcast_utm_transform`** and set to true to add the UTM frame. This parameter has to be set to true in order to use **`broadcast_utm_transform_as_parent_frame`**. 
-   Notes:
    -   **`yaw_offset`**: This parameters should be set to 0 when the SVEA is facing **EAST**. If not, this parameters should be changed in order to work with RTK-GPS.
### rs_odometry.launch
-   Changes:
    -   remapped the topic **`/set_pose`** to **`/global/set_pose`** to distinguish it from the one for EKF local.

## Outdoor autonomous driving with RTK-GPS
-   An example to demonstrate the performance of the updated outdoor localization stack (with RTK-GPS).
### outdoor_autonomous_driving.launch
-    A launch file that include the localiza.launch, relative_waypoints.py and pure_pursuit_outdoor.py
### relative_waypoints.py
-   A node that take in the GPS location of the points (defined by users) and transform them to the map coordinates based on the initial pose of the robot. These coordinates are then published to **`/outdoor_localization_waypoint`**. 
### pure_pursuit_outdoor.py
-   A node that has a very similar functionalities as the pure_pursuit.py, but it subscribes to **`/outdoor_localization_waypoint`** and use these points to generate the trajectory for pure pursuit. 
### How to run:
```
roslaunch svea_examples outdoor_autonomous_driving.launch is_indoors:=false start_serial:=true device:=/dev/GPS
```
-   Parameters:
    -   **`device`**: the port for the RTK-GPS. If the script is running on SVEA2, the port is fixed to `/dev/GPS`. Otherwise, change it accordingly.
    -   **`use_wheel_encoders`**: true if the script is running on SVEA7.
    -   **`initial_pose_x`**, **`initial_pose_y`**, **`initial_pose_a`**: initial pose of SVEA (x,y,theta)
    -   **`yaw_offset`**: This parameters should be set to 0 when the SVEA is facing **EAST**. If not, this parameters should be changed in order to work with RTK-GPS.
-   Notes:
    -   It might take a few trials before the RTK-GPS connects to the service.
### Performance:
-   The SVEA can trace the path, at least the outline of the path accuractely, given good accuracy of the RTK-GPS. However, since the RTK-GPS drifts, even if the SVEA drives to those points according to rViz, it might not be driving on that exact same place in the physical environment.
-   The first image shows the expected path in orange, and the second image shows the actual path the SVEA took. 
-   The 4 blue points in the images are the inputs for the relative_watpoints.py, and the path is generated using these 4 points. But the entire path drifted to one direction. 

![RTK_GPS_Autonomous_driving_expectation](https://github.com/KTH-SML/svea/assets/103376915/7f9d4369-cacc-4511-a52d-b65de25c7044)
![RTK_GPS_Autonomous_driving](https://github.com/KTH-SML/svea/assets/103376915/325b3d98-02a3-45da-82b1-0d2812404b60)


# svea_navigation_teb
The outdoor navigation stack is meant to provide tools that enables safe autonomous navigation on sidewalks for the SVEA platform. 
The navigation relies on the correct installation of svea_sensors (GPS outdoor localization) and svea_vision (dynamic obstacles detection and processing).
The current implementation incorporates the move_base ROS package, with TEB as local planner and NavfnROS as global one.


## Installation with docker

You must have docker installed.

```
util/build
util/create
util/start
```

## Usage
```
roslaunch svea_navigation_teb navigation.launch
```


## Current achievements
Smoothly works on simulation.
Has multiple limitations on the real car because of the problematics later explained.


## Problematics
### Mechanical set-up
Camera vibrations
### Sensor limits
Nearby obstacles are not identified by vision system.
YOLO is not capable of identifying every object (chairs for instance).
Lidar can not be currentely employed on SVEA ISMIR because the vehicle leans forward too much.
### Software limits
Image processing intoduce considerable delays, up to 0.3 seconds.



## Next steps
### TEB
Incorporate Reachability Analysis.
Manage unplanned safety situations.
Add logic to stop SVEA when a forward trajectory is not found.
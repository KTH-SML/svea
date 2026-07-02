# svea_mocap

This repository contains convenience launch files and python class for
integrating the Qualisys Motion Capture (mocap) System into the SVEA software
stack.

## Installation

Please refer to [svea](https://github.com/KTH-SML/svea) on how to install and
run SVEA software.

Included in the base docker image (`ghcr.io/kth-sml/svea`), is the driver 
[motion\_capture\_system](https://github.com/DISCOWER/motion_capture_system)
for ROS-ifying data from the mocap.

## Usage

For setting up the motion capture system, contact one of the lab's
research engineers who works with the system.

**Note:** For the following to work, you need to be on the lab network.

Once you have gotten help to set up your environment on the motion
capture system, you can proceed to launch:

```bash
ros2 launch svea_mocap mocap.launch.py
```


#!/bin/bash
DEVICE=$(ls /dev/serial/by-id/*SVEA-LLI* | head -1)
<<<<<<< HEAD
ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEVICE" -b 115200
=======
ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEVICE" -b 1000000
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))

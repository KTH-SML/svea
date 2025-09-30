#!/bin/bash
DEVICE=$(ls /dev/serial/by-id/*SVEA-LLI* | head -1)
ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEVICE" -b 1000000

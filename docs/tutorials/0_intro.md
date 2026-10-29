# Software Overview

SVEA runs ROS 2 Jazzy. The same Python interfaces connect controllers to
simulated and physical vehicles.

For the vehicle itself, see [hardware and assembly](../setup/hardware.md)
and [RC operation](../setup/rc_operation.md).

## Workspace layout

| Package | Purpose |
| --- | --- |
| `svea_core` | Controllers, interfaces, models, simulation and PX4 communication. |
| `svea_examples` | Example controllers and launch files. |
| `svea_localization` | Localization, sensor drivers and mapping launches. |
| `svea_mocap` | Launch wrapper for DISCOWER's Qualisys driver. |

## Controller interfaces

A controller reads position and velocity through `LocalizationInterface` and
sends steering and velocity through `ActuationInterface`.

- Steering is in radians; velocity is in m/s.
- The default localization topic is `odometry/local` within the vehicle namespace.
- Hardware commands reach PX4 through MAVROS.

See the [interface reference](../reference/interfaces.md) for parameters and
methods, and [ROS connection](../setup/ros_bringup.md) for hardware bringup.

## Rosonic

The examples use [Rosonic](../reference/rosonic.md), a layer over `rclpy`
that declares publishers, subscribers, parameters and timers as Python class
members. Standard ROS 2 nodes can also use the workspace.

Next: [modify the Floor 2 controller](1_floor2.md).

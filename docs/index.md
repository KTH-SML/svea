# SVEA

ROS 2 Jazzy software and operating instructions for Small Vehicles for Autonomy.

## Start here

- **Run a simulation:** follow [Docker setup](setup/docker.md), then the [Floor 2 tutorial](tutorials/1_floor2.md).
- **Assemble a vehicle:** see [hardware and assembly](setup/hardware.md).
- **Drive with the RC:** follow [RC operation](setup/rc_operation.md).
- **Connect ROS to a vehicle:** follow [ROS connection](setup/ros_bringup.md).
- **Set up tracking:** see [MoCap localization](mocap_localization.md) or [LiDAR setup](setup/lidar_setup.md).

## Develop a controller

Start with the [software overview](tutorials/0_intro.md). The
[examples](examples/pure_pursuit.md) show how to combine the
[interfaces](reference/interfaces.md), [models](reference/models.md) and
[Rosonic](reference/rosonic.md).

## Related repositories

- [svea_parts](https://github.com/KTH-SML/svea_parts): CAD files, printable parts and assembly manuals.
- [SVEA-PX4-Autopilot documentation](https://github.com/nilskiefer/SVEA-PX4-Autopilot/blob/8e17861fa85809fecdbdddea035b0fa49264517a/docs/en/svea/index.md): firmware setup, flashing, troubleshooting and PMB3 reference.

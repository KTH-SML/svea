# Teleoperation Examples

The examples provide `twist_consumer.py` and `joy_consumer.py`. Start vehicle
simulation or [hardware bringup](../setup/ros_bringup.md) separately, then use
the same vehicle namespace for the command source and consumer. Stop any
existing driving controller before starting teleoperation.

For a simulation called `self`, start the twist consumer in another sourced
terminal:

```bash
ros2 run svea_examples twist_consumer.py --ros-args -r __ns:=/self
```

It subscribes to `/self/cmd_vel` (`geometry_msgs/msg/Twist` by default),
converts linear velocity in m/s and yaw rate in rad/s into steering/velocity
commands, and publishes through `ActuationInterface` on
`/self/mavros/manual_control/send`. Its command timeout defaults to 0.6 seconds.
Configure any keyboard or navigation publisher to use that same topic.

The [joystick consumer](https://github.com/KTH-SML/svea/blob/main/src/svea_examples/scripts/joy_consumer.py)
expects `sensor_msgs/msg/Joy` from a separate joystick driver. Inspect its
`joy_kind`, axis mappings and button behavior before using a controller.
It is a separate example and does not inherit the twist consumer's timeout.
Keep the physical RC override available on hardware.

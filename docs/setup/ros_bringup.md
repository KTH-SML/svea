# Connect ROS to the Vehicle

Use the onboard Linux computer for these steps. For driving without ROS,
use [RC operation](rc_operation.md).

## Start the container

Follow steps 1–2 of [RC startup](rc_operation.md#start) to power the RC and
vehicle. Keep the RC switch in **kill** while checking the connection. Connect the PX4 controller's **CN1** USB port to the
onboard computer; CN2 is the flashing port.

From the host checkout:

```bash
util/build
util/run
```

On Linux, `util/run` gives the container access to host devices and networking.

## Connect to PX4

Inside the container, find the runtime serial device:

```bash
ls /dev/serial/by-id/*SVEA*
```

Start MAVROS using the default device and baud rate:

```bash
ros2 launch svea_core lli.xml name:=self
```

The default is `/dev/serial/by-id/usb-SVEA_PX4_AUTOPILOT_0-if00` at 921600 baud.
If the listed device differs, pass its path as `serial_device:=/dev/serial/by-id/DEVICE`.

Open another host terminal and run `util/exec`. Inside that shell:

```bash
ros2 topic echo /self/mavros/state --once
```

Check that `connected` is `true`. This launch starts the PX4 connection and
telemetry tunnel; it does not start a driving controller or localization.

## Add localization and a controller

Configure [LiDAR](lidar_setup.md) or [MoCap](../mocap_localization.md) for the
vehicle and map. See the [localization launch reference](https://github.com/KTH-SML/svea/blob/main/src/svea_localization/README.md)
for available sensors and frames.

For a combined launch, `svea_core/svea.launch.py` includes MAVROS and localization
when `is_sim=False`. Configure its serial connection, sensors and map for the
vehicle, and stop the standalone `lli.xml` launch before starting it.

Start your tested controller only after localization reports the vehicle's
correct position and heading. The RC switch selects **left = ROS**,
**middle = RC**, **right = kill**. The [RC guide](rc_operation.md#controls)
shows the switch and arm button.

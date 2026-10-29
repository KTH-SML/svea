# PX4 uORB Tunnel Bridge

The custom PX4 `PX4_UORB_TUNNEL` stream carries selected uORB telemetry in
MAVLink `TUNNEL` messages using payload type `0xE001`. MAVROS delivers these to
ROS 2; `svea_core/scripts/px4_uorb_tunnel.py` decodes the stream and publishes
available message types. This feature depends on the deployed SVEA firmware;
it is not a stream supplied by every stock PX4 build.

## Normal bringup

`svea_core/launch/lli.xml` starts MAVROS and the tunnel bridge together under
the vehicle namespace (`self` by default). It configures the bridge's
`mavlink_topic` as relative `mavros/tunnel/out`. Do not start another bridge
against the same stream if vehicle bringup already supplies it.

For a separately established MAVROS connection, the installed executable is:

```bash
ros2 run svea_core px4_uorb_tunnel.py --ros-args -r __ns:=/self
```

The workspace has no Docker Compose bringup to rely on. Source the workspace
and check the actual running node and topic names.

## Topics and message definitions

Outputs are **relative** to the bridge's namespace. With `/self`:

| Topic | Meaning |
| --- | --- |
| `/self/px4/uorb_tunnel/frame` | Decoded frame metadata as `std_msgs/msg/String` JSON, useful for debugging. |
| `/self/px4/uorb/power_monitor` | Available power-monitor messages multiplexed across instances. |
| `/self/px4/uorb/power_monitor/instance_0` | One instance; additional instance topics appear as the firmware sends them. |

A root-namespace standalone bridge instead publishes under `/px4/...`.
The bridge needs matching ROS message definitions; it searches the packages
selected by `ros_msg_packages` (default `px4_msgs,svea_msgs`) and can accept an
`override_msg_dir`. Match definitions to the actual firmware and custom uORB
types. Do not assume upstream PX4 definitions include all SVEA custom messages.

Instance numbers are assigned by the firmware and board configuration. Check
the deployed board's sensor inventory before interpreting an instance as a
particular voltage rail; this ROS package does not guarantee that mapping.

## Verification

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /self/mavros/tunnel/out
ros2 topic echo /self/px4/uorb_tunnel/frame --once
ros2 topic hz /self/px4/uorb/power_monitor/instance_0
```

If no data arrives, verify MAVROS `state.connected`, the tunnel plugin, the
custom stream configuration on the correct MAVLink link, schema compatibility,
and bridge diagnostics. Use the deployed firmware's startup instructions and
NSH `mavlink help` rather than assuming a UDP port or serial device. Debugging
telemetry does not establish that motion control is armed or safe.

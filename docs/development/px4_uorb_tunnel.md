# PX4 uORB Tunnel Bridge

## Overview
The `px4_uorb_tunnel` bridge allows forwarding of internal PX4 `uORB` topics directly to ROS 2 over MAVLink. This is typically used for custom or high-bandwidth telemetry that does not map cleanly to standard MAVLink messages. In the SVEA architecture, this is specifically used to retrieve detailed rail-level telemetry from the custom SVEA power board (`power_monitor` topics).

## Architecture
1. **PX4 firmware (Sender):** The `PX4_UORB_TUNNEL` stream captures selected local uORB topics, serializes them (handling fragmentation if necessary), and sends them over MAVLink using the `TUNNEL` message (ID 128, payload type `0xE001`).
2. **MAVROS (Transport):** The standard `mavros_node` relays the raw MAVLink `TUNNEL` payload to ROS 2.
3. **SVEA Core Bridge (Receiver):** The `svea_core` package provides a `px4_uorb_tunnel` node. It subscribes to the tunnel stream, decodes the custom binary payload, reconstructs fragmented messages, and republishes them as native ROS 2 topics under the `/px4/uorb/*` namespace.

## Exposed ROS 2 Topics

When the tunnel bridge is active, you will see topics dynamically populate based on what the PX4 board is forwarding:

### Raw Tunnel Interface
- `/px4/uorb_tunnel/frame`: Raw extracted uORB frames decoded from the MAVLink tunnel (primarily for debugging).

### Power Monitor Telemetry
The SVEA power board exposes multiple voltage and current sensors (INA226 and INA3221). These are surfaced in ROS 2 as:

- `/px4/uorb/power_monitor`: Contains all power monitor updates multiplexed.
- `/px4/uorb/power_monitor/instance_X`: Dedicated topics for each physical sensor/channel instance, guaranteeing isolation between rails.

#### Instance Determinism
The PX4 SVEA firmware guarantees deterministic ordering for the power monitor instances. For example, on the STM32F7 Clicker4:
- `instance_0` & `instance_1`: INA226 devices (e.g., ESC current shunt, Servo buck shunt).
- `instance_2` through `instance_7`: INA3221 channels (e.g., 5V/12V rails, auxiliary inputs).

## Quick Start

### 1. Check PX4 Configuration
The board's startup script (`rc.board_extras`) usually configures this automatically. If you need to manually enable the `PX4_UORB_TUNNEL` stream from the PX4 NSH console:
```bash
mavlink stream -u 0 -s PX4_UORB_TUNNEL -r 20
```

### 2. Run the ROS 2 Bridge
Assuming your MAVROS connection is already established (e.g., via the SVEA Docker compose setup):

```bash
ros2 run svea_core px4_uorb_tunnel
```

*(Note: The SVEA Docker compose environment may already start this automatically in the background).*

### 3. Verify Data
Use standard ROS 2 commands to verify that the tunnel is correctly decoding the uORB topics:

List the active topics:
```bash
ros2 topic list | grep px4
```

Echo a specific power rail instance:
```bash
ros2 topic echo /px4/uorb/power_monitor/instance_0
```
```bash
ros2 topic hz /px4/uorb/power_monitor/instance_0
```

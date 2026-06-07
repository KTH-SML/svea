# MAVLink Routing

SVEA uses MAVROS to communicate with the PX4 autopilot. The launch file
`svea_core/launch/svea.xml` supports both direct USB serial MAVROS and
MAVProxy-based fanout for tools such as QGroundControl.

## Direct MAVROS over USB

Use this when MAVROS is the only process that should own the PX4 USB serial
device:

```bash
ros2 launch svea_core svea.xml is_sim:=false use_mavproxy:=false
```

This starts MAVROS with:

```text
serial://$(var lli_serial_device):$(var lli_baud_rate)
```

The defaults are:

```text
lli_serial_device=/dev/serial/by-id/usb-SVEA_PX4_AUTOPILOT_0-if00
lli_baud_rate=921600
```

Override the serial device when needed:

```bash
ros2 launch svea_core svea.xml \
  is_sim:=false \
  use_mavproxy:=false \
  lli_serial_device:=/dev/ttyACM0 \
  lli_baud_rate:=921600
```

QGroundControl cannot directly open the same USB serial device while MAVROS
owns it.

## MAVProxy Inside The Container

This is the default non-simulation path:

```bash
ros2 launch svea_core svea.xml is_sim:=false use_mavproxy:=true
```

`svea.xml` starts MAVProxy on the serial device and forwards MAVLink to MAVROS
on UDP `14551`. If `mavproxy_qgc` is true, it also forwards to QGroundControl
on `$(var qgc_host):14550`.

## Host MAVProxy On macOS

Docker Desktop for macOS does not provide Linux-style host networking. To use
QGroundControl on the macOS host and MAVROS inside the SVEA container at the
same time, run MAVProxy on the host and launch SVEA without starting a second
MAVProxy instance inside the container.

On the macOS host:

```bash
mavproxy.py \
  --master=/dev/cu.usbmodemXXXX,921600 \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551
```

QGroundControl receives MAVLink on `14550`. The SVEA container receives MAVLink
on published UDP port `14551`, where MAVROS binds `udp://:14551@`.

Inside the container:

```bash
ros2 launch svea_core svea.xml is_sim:=false use_mavproxy:=true start_mavproxy:=false
```

The SVEA Docker run script publishes UDP `14551` on macOS. If running Docker
manually, include:

```bash
-p 14551:14551/udp
```

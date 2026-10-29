# LiDAR Setup

The hardware launch file `src/svea_localization/launch/sensors/lidar.launch.py`
starts `urg_node_driver` with a default `lidar_ip` of `192.168.0.10`. Confirm
the address configured on the actual Hokuyo before changing network settings.

For a sensor at `192.168.0.10/24`, configure the computer's dedicated Ethernet
interface to an unused address in the same subnet, for example
`192.168.0.100` with netmask `255.255.255.0`. A direct sensor connection does
not require a gateway or DNS. Keep your separate internet/lab connection intact.
On Ubuntu Desktop, set this under Wired Settings → IPv4 → Manual, apply,
and reconnect that interface.

Check the interface and sensor from the host:

```bash
ip address
ping 192.168.0.10
nc -zv 192.168.0.10 10940
```

Run the driver inside a sourced, hardware-access container:

```bash
ros2 launch svea_localization lidar.launch.py lidar_ip:=192.168.0.10
```

A standalone driver publishes `scan` in its root namespace. When included by
`localization.launch.py` under vehicle name `self`, the topic is `/self/scan`:

```bash
ros2 topic info /self/scan
ros2 topic hz /self/scan
```

The main `svea.launch.py` does not expose a `lidar_ip` argument; configure the
localization include in a custom Better Launch file for a different address.
For simulated scans, see the [LiDAR simulation tutorial](../tutorials/2_lidar_sim.md).

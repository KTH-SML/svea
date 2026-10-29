# Listening to ROS 2 on Another Computer

Use Foxglove for visualization, or DDS discovery to connect ROS 2 nodes
across computers.

## Foxglove over a WebSocket

For visualization without a ROS installation on the laptop, start a Foxglove
bridge on the SVEA and connect the host app/browser to `ws://SVEA_IP:8765`.
`floor2.launch.py` starts this bridge by default for simulation. The
hardware/simulation stack in `svea.launch.py` can include it with
`use_foxglove:=True`. Development containers publish port 8765; hardware
containers on native Linux use host networking.

## Direct DDS communication

Both computers need compatible ROS 2 installations and message definitions.
For the default DDS middleware, set the same agreed domain in every terminal
that starts nodes on either computer:

```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

42 is an example; use the domain assigned to your experiment. The base Docker
image sets discovery to `LOCALHOST`, so override it **inside** the container
before launching nodes. Exporting variables only on the host does not pass
them through `util/run` automatically. Restart nodes and the ROS CLI daemon
after changing discovery settings:

```bash
ros2 daemon stop
ros2 topic list
ros2 node list
```

The hosts must permit DDS traffic and multicast, or have explicit middleware
peer configuration. Host networking on native Linux avoids Docker bridge NAT;
macOS/Windows virtualization and WiFi policy can limit discovery. Publishing
only the Foxglove port does not enable DDS discovery across hosts.

See the ROS 2 Jazzy [discovery documentation](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Improved-Dynamic-Discovery.html)
for `ROS_STATIC_PEERS` and middleware-specific limitations. The discovery
variables above do not configure `rmw_zenoh`.

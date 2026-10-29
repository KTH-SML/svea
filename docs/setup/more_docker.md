# Advanced Docker Workflows

For installation, starting a container and rebuilding the workspace, use
[Docker setup](docker.md).

## Create a package

Inside the container, from `/svea_ws/src`:

```bash
ros2 pkg create --build-type ament_python my_controller
```

Declare ROS dependencies in `package.xml`. Register scripts, launch files and
data files in `setup.py`, following `svea_examples`. Then
[build and source the workspace](docker.md#edit-and-rebuild).

## Visualization

Foxglove connects through port 8765 and needs no display forwarding.
To use RViz, rqt or the obstacle editor, run them in a graphical ROS 2
environment on the host, or configure display forwarding for your container
runtime. `util/run` does not configure X11 or Wayland forwarding.
See [remote ROS 2 access](../development/remote_ros.md) for connecting host tools.

## Container access on Linux

`util/run` uses host networking and device access on Linux. For a container
without device access and with published ports instead of host networking, use
`DEV=1 util/run`. macOS selects that configuration automatically.

This setting controls Docker access and networking. Floor 2 runs in simulation
by default through its separate `is_sim=True` launch argument.

## Multiple containers

Independent experiments need distinct container names and host port mappings.
Inspect the generated Docker command with `DEBUG=1 util/run` before
adapting it; two containers cannot bind the same host port.

Use different `ROS_DOMAIN_ID` values to isolate experiments. Containers that
must communicate need the same domain and working DDS discovery; see
[remote ROS 2 access](../development/remote_ros.md).

## Image maintenance

See [maintaining Docker images](../development/docker.md) for architecture
selection, base images and registry publishing.

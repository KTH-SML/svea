# Building the SVEA Stack Natively

## Requirements

The native target is Ubuntu 24.04 with ROS 2 Jazzy. Install ROS and development
tools using the [Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
The [Docker workflow](../setup/docker.md) supplies these dependencies without
modifying the host Python installation.

## Workspace and external dependencies

Clone the workspace before running commands that use `requirements.txt`:

```bash
git clone https://github.com/KTH-SML/svea.git
cd svea
source /opt/ros/jazzy/setup.bash
```

The base Dockerfile also builds external source dependencies. Native users
must provide compatible installations of Better Launch (`v1.6.0`), PX4 message
definitions (`v1.16.1`), and the DISCOWER Qualisys driver for mocap. To mirror
those source dependencies in this workspace:

```bash
git clone --branch v1.6.0 https://github.com/dfki-ric/better_launch.git src/better_launch
git clone --branch v1.16.1 --depth 1 https://github.com/PX4/px4_msgs.git src/px4_msgs
git clone https://github.com/DISCOWER/motion_capture_system.git src/motion_capture_system
```

Do not clone duplicate packages if you already have a sourced dependency
workspace. See `docker/Dockerfile.base` for the current dependency versions,
MAVROS installation and GeographicLib dataset setup.

Install the workspace dependencies using an initialized rosdep installation:

```bash
rosdep update --rosdistro jazzy
rosdep install --rosdistro jazzy --from-paths src --ignore-src -r -y
python3 -m pip install --break-system-packages -r requirements.txt
```

The pip command mirrors the image's system-Python installation on Ubuntu;
understand its effect on the native machine before using it. The NumPy
constraint in `requirements.txt` is needed for compatibility with ROS binary
packages. Do not remove it to work around an installation conflict.

Build and source the workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

Source `install/setup.bash` in every terminal that runs SVEA. An optional
persistent Bash setup is:

```bash
echo 'source /absolute/path/to/svea/install/setup.bash' >> ~/.bashrc
```

Replace the absolute path before running that command. Rebuild after adding
scripts, launch files, package metadata or installed data, then source the
updated overlay. Test with `ros2 launch svea_examples floor2.launch.py` before
moving to hardware. A native install does not configure USB permissions,
sensor networking, QTM calibration, or vehicle firmware automatically.

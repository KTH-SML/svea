# Docker Setup

Docker supplies ROS 2 Jazzy and the workspace dependencies. Choose a container runtime:

- **macOS:** we recommend [OrbStack](https://orbstack.dev/). Docker Desktop also works.
- **Linux:** install [Docker Engine](https://docs.docker.com/engine/install/).
- **Windows:** install [Docker Desktop](https://docs.docker.com/desktop/) with its WSL 2 backend.

## Build and start

Run on the host:

```bash
git clone https://github.com/KTH-SML/svea.git
cd svea
util/build
util/run
```

If you already cloned the repository, run the last two commands from its root.
`util/run` detects macOS and publishes Foxglove port 8765. On Linux it
uses host networking and enables device access. The launch file selects
simulation or hardware independently of these container settings.

Inside the container:

```bash
ros2 launch svea_examples floor2.launch.py
```

The image builds and sources the workspace. The launch starts two simulated
vehicles, `svea_a` and `svea_b`, following paths on the Floor 2 map.

## View the simulation

Open [Foxglove](https://foxglove.dev/download) on the host and connect to
`ws://localhost:8765`. Choose **Layout → Import from file** and select
`foxglove/Floor2 Pure Pursuit.json` from the checkout. For a remote container,
replace `localhost` with its host's address.

Continue with the [Floor 2 tutorial](../tutorials/1_floor2.md).

## Edit and rebuild

Edit files under `src/` on the host. That directory is mounted at
`/svea_ws/src` in the container. After adding files or changing package metadata,
launch files or installed assets, run inside the container:

```bash
colcon build --symlink-install
source install/setup.bash
```

For a focused build, add `--packages-up-to svea_examples` to the build command.
Rebuild the image with `util/build` after changing dependencies or Dockerfiles.

To open another terminal in the running container, run on the host:

```bash
util/exec
```

## Stop and restart

Press Ctrl+C to stop the launch. Exit the container with `exit` or Ctrl+D.
`util/run` removes the container on exit: edits under `src/` persist, but
container-local build results and other files are lost. Start again with
`util/run`.

For a physical vehicle, use [ROS connection](ros_bringup.md). For GUI tools
and multiple containers, see [advanced Docker workflows](more_docker.md).

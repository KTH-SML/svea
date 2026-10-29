# LiDAR Simulation

The simulator represents obstacles as polygons in map coordinates. The current
`simulation.launch.py` reads `<map_name>.obstacles.yaml` from the selected map
package and starts `sim_lidar.py` when that file contains obstacles.

## Obstacle editor

Run the Tk editor from the repository root in a graphical environment:

```bash
python3 src/svea_core/util/obstacles_builder_gui.py
```

The editor imports ROS message definitions and needs a sourced ROS environment,
Tk, NumPy, Pillow and PyYAML. Install missing GUI dependencies on the machine
where you run it. `util/run` does not automatically configure display
forwarding; see [visualization options](../setup/more_docker.md#visualization).

Use **Load map** to select `src/svea_core/maps/floor2.yaml` (with its referenced
PGM image). The editor also accepts legacy pickled `OccupancyGrid` maps; only
load trusted pickle files. It supports zoom/pan, undoing points, clearing
polygons and simplifying them. Coordinates exported after zoom/pan remain in
the map frame.

Draw polygons, finish each obstacle, then choose an **obstacles** export format
and save as `floor2.obstacles.yaml` alongside the map. Both the plain form
below and the editor's ROS parameter wrapper are accepted by the launch loader:

```yaml
obstacles:
  - [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
```

Do not repeat the first vertex; the simulator closes the polygon. The editor's
`stay_in`/`keep_out` track formats are separate outputs and are not the
obstacle list consumed by this launch file. Keep copies of the supplied maps
and obstacles if you need to preserve the original course layout.

## Run and inspect

After editing installed map data, rebuild the map package and source the
workspace. The supplied Floor 2 obstacle file enables scans for both vehicles:

```bash
ros2 launch svea_examples floor2.launch.py
```

In another sourced terminal in the same container/domain:

```bash
ros2 topic info /svea_a/scan
ros2 topic hz /svea_a/scan
ros2 topic echo /svea_a/scan --once
```

`scan` is a `sensor_msgs/msg/LaserScan` topic relative to the vehicle namespace.
The simulator also publishes `viz_lidar_points`, `viz_lidar_rays`, and
`viz_edges`. Enable the relevant topics in Foxglove's 3D panel; they may be
hidden in the supplied layout. A controller must explicitly subscribe to scans;
`LocalizationInterface` does not provide a scan callback automatically.

To test another map, provide its YAML/PGM and matching obstacle file through
the selected map package and include `simulation.launch.py` with that
`map_name`. `floor2.launch.py` itself fixes the map name to `floor2`.

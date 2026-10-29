# Floor 2 and Pure Pursuit

Modify the supplied path follower to reduce overshoot, stop for obstacles and
park at a chosen position. Run these exercises in simulation.

## Start the example

Complete [Docker setup](../setup/docker.md), including the Foxglove layout.
The Floor 2 launch starts two vehicles, `svea_a` and `svea_b`.

Inspect these files in the checkout:

- `src/svea_examples/launch/floor2.launch.py`: vehicles, map and follower configuration.
- `src/svea_examples/scripts/static_path_follower.py`: reads vehicle state, selects goals and sends commands.
- `src/svea_core/svea_core/controllers/pure_pursuit.py`: steering and velocity calculations.

Create an `ament_python` package using [these steps](../setup/more_docker.md#create-a-package).
Copy the launch and follower into your package, register both in `setup.py`,
and change the copied launch to run your package's follower. Build and source
using the [development workflow](../setup/docker.md#edit-and-rebuild).
Stop the supplied launch before running your copy, so each vehicle has one controller.

## 1. Reduce overshoot

Watch the vehicle through several turns and identify where it cuts corners
or approaches walls. Read `compute_steering()` and `_calc_target_index()` in
`PurePursuitController` to see how it selects and tracks a look-ahead point.

Adjust your controller to reduce overshoot. Compare runs on the same path and
at the same target speed, then test how the result changes with speed.

## 2. Stop for obstacles

Represent obstacles as map-frame XY coordinates, for example
`[(x1, y1), (x2, y2)]`. Make your controller stop when the vehicle approaches
an obstacle. Choose and document the stopping distance.

Display the obstacles in Foxglove using the marker helpers in
`src/svea_core/svea_core/interfaces/marker.py` and
`src/svea_core/svea_core/utils/viz_util.py`.

Extensions:

- Add obstacle points through Foxglove's **Publish Point** tool on `/clicked_point`.
- Use the [LiDAR simulation](2_lidar_sim.md) to detect obstacles from scans.

## 3. Park at a goal

The supplied follower cycles through its waypoints. Change your copy to stop
at the final waypoint instead of selecting the first one again.
`PurePursuitController.is_finished` signals completion of its current trajectory;
the follower's `update_goal()` currently starts the next one.

Define a position tolerance for parking and check the final position in
Foxglove. Extend the controller to meet a heading tolerance as well, then
combine parking with your obstacle stop.

## Inspect the result

Open a second container shell with `util/exec` on the host. Inside it:

```bash
ros2 topic echo /svea_a/odometry/local --once
ros2 topic hz /svea_a/scan
```

Use Foxglove to compare the path, vehicle position and obstacles. Check both
vehicles if your launch still starts two.

For hardware setup, use [ROS connection](../setup/ros_bringup.md).

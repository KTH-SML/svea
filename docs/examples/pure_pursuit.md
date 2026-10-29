# Pure Pursuit Controller Example

Inside a built and sourced workspace:

```bash
ros2 launch svea_examples floor2.launch.py
```

The launch defaults to simulation and starts two vehicles, `svea_a` and
`svea_b`, each running `static_path_follower.py`. It supplies flattened XY
waypoints through `points` and a target velocity of 0.4 m/s. The follower uses
`PurePursuitController`, `ActuationInterface`, and `LocalizationInterface`.

See the [Floor 2 tutorial](../tutorials/1_floor2.md), the
[follower source](https://github.com/KTH-SML/svea/blob/main/src/svea_examples/scripts/static_path_follower.py),
and the [controller source](https://github.com/KTH-SML/svea/blob/main/src/svea_core/svea_core/controllers/pure_pursuit.py).

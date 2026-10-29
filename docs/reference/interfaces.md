# Interfaces

The installed Python package is `svea_core`; import the public interfaces with:

```python
from svea_core.interfaces import ActuationInterface, LocalizationInterface, ShowMarker, ShowPath
```

Declare these resources as fields on an `rx.Node`, as in the
[static path follower](https://github.com/KTH-SML/svea/blob/main/src/svea_examples/scripts/static_path_follower.py).
Rosonic starts them with their owning node.

| Interface | Current behavior |
| --- | --- |
| `ActuationInterface` | `send_control(steering, vel_or_acc)` accepts radians and m/s by default; `use_acceleration=True` changes the second input to acceleration. Commands are published periodically to the relative `mavros/manual_control/send` topic. |
| `LocalizationInterface` | Subscribes to relative `odometry/local`, transforms pose to `map` and twist to the configured base frame, and returns `(x, y, yaw, velocity)` from `get_state()`. |
| `ShowMarker` | Publishes visualization markers, including a goal marker through `place()`. |
| `ShowPath` | Publishes a path through `publish_path(xs, ys)`. |

Localization parameters are `localization/map_frame`, `localization/base_frame`,
and `localization/odom_top`; defaults are `map`, `self/base_link`, and
`odometry/local`. Match the base frame to your vehicle namespace. For mocap,
see the [driver integration guide](../mocap_localization.md).

See the [interface sources](https://github.com/KTH-SML/svea/tree/main/src/svea_core/svea_core/interfaces)
for signatures and docstrings. The physical RC and firmware safety mechanisms
remain necessary; the actuation interface is not itself a deadman controller.

# Models

Models live in the installed `svea_core.models` package:

```python
from svea_core.models.bicycle import Bicycle4D, Bicycle4DWithESC
from svea_core.models.platooning import C_OVRV
```

`Bicycle4D` stores `(x, y, yaw, velocity)` and integrates steering angle and
acceleration inputs with `update(delta, accel, dt=None)`. Units are metres,
radians, seconds and m/s. `Bicycle4DWithESC` accepts steering and target
velocity, adding a first-order ESC response before the bicycle update.

`C_OVRV` is the cooperative longitudinal platooning model; its constructor
requires platoon/communication parameters rather than a single vehicle pose.
See the [bicycle](https://github.com/KTH-SML/svea/blob/main/src/svea_core/svea_core/models/bicycle.py)
and [platooning](https://github.com/KTH-SML/svea/blob/main/src/svea_core/svea_core/models/platooning.py)
sources for constructor arguments, equations, units and assumptions.

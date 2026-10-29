# Rosonic

Rosonic is the workspace's declarative layer over `rclpy`:

```python
from svea_core import rosonic as rx
from std_msgs.msg import String

class Example(rx.Node):
    output = rx.Publisher(String, 'status')
    label = rx.Parameter('ready')

    @rx.Timer(1.0)
    def publish_status(self):
        self.output.publish(String(data=self.label))

if __name__ == '__main__':
    Example.main()
```

`rx.Node.main()` initializes ROS, starts declared resources and spins the node.
`rx.Field` groups resources; `rx.namespace` groups parameter names.
`rx.Parameter` declares a ROS parameter with its default type.
`rx.Publisher`, `rx.Subscriber` and `rx.Timer` register communication and timer
resources. `on_startup()` and `on_shutdown()` provide lifecycle hooks.

Use standard `--ros-args` options for namespaces, remappings and node parameters.
For Better Launch function arguments passed through `ros2 launch`, spell
boolean overrides `True` or `False`, for example `is_sim:=False`. Its adapter
evaluates Python literals; lowercase values can remain strings. In Python
launch code, pass booleans directly (`is_sim=False`).
The [source](https://github.com/KTH-SML/svea/blob/main/src/svea_core/svea_core/rosonic.py)
contains further examples and the exact supported signatures.

#! /usr/bin/env python3

from tf_transformations import quaternion_from_euler

from svea_core.interfaces import LocalizationInterface
from svea_core.interfaces import ActuationInterface
<<<<<<< HEAD
from svea_core.utils import ShowPath
=======
>>>>>>> 9585fc8 (Teleop control in simulation with teleop_twist_keyboard added)
from svea_core import rosonic as rx
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

<<<<<<< HEAD

=======
>>>>>>> 9585fc8 (Teleop control in simulation with teleop_twist_keyboard added)
qos_subber = QoSProfile(depth=10 # Size of the queue 
                        )

class teleop_control(rx.Node):  # Inherit from rx.Node
<<<<<<< HEAD
    """ Teleoperation control node for Svea.

    #**Background**

    A simple teleoperation control node which helps users understand how to
    setup the node for svea and send control commands to the vehicle.

    #**Preparation**

    TODO: Add instructions for setting up the teleoperation environment.

    #**Task**

    ##**1. Start launch file (simulation)**

    To launch teleop example, you can use the following command:
    ```bash
    ros2 launch svea_examples teleop_floor2.xml is_sim:=true
    ```

    This launch file includes three parts:
    - `teleop_control`: The teleoperation control node.
    - `svea_launch`: The launch file for the SVEA vehicle, which includes the simulation and the low-level interface.
    - 'map_and_foxglove': The launch file for the map and foxglove visualization.

    ##**2. Control the robot**

    Beside launch the teleope launch file, you also need to run controller node, for example:
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
    This will allow you to control the robot using the keyboard.

    Note:
        You can also use different teleoperation packages, such as `teleop_twist_keyboard` or `joy`, to control the robot.

    ##**3. Launch on svea**
    
    To launch the teleoperation control node on the SVEA vehicle, you can use the following command:
    ```bash
    ros2 launch svea_examples teleop_floor2.xml is_sim:=false
    ```

    Attributes:
        DELTA_TIME (float): Time interval for control loop.
        target_velocity (float): Target velocity for the robot.
        actuation (ActuationInterface): Interface for sending control commands.
        localizer (LocalizationInterface): Interface for localization and state retrieval.
        path (ShowPath): Interface for visualizing the path.
    """

    DELTA_TIME = 0.1

    target_velocity = rx.Parameter(1.0)

    actuation = ActuationInterface()
<<<<<<< HEAD
=======
    localizer = LocalizationInterface()
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))

    # Path Visualization
    path = ShowPath()

=======

    DELTA_TIME = 0.1

    is_sim = rx.Parameter(True)
    target_velocity = rx.Parameter(1.0)

<<<<<<< HEAD
>>>>>>> 9585fc8 (Teleop control in simulation with teleop_twist_keyboard added)
=======
    actuation = ActuationInterface()
    localizer = LocalizationInterface()

>>>>>>> 01b7a7d (teleop example bug fixed)
    ## Subscribers ##
    @rx.Subscriber(Twist, 'cmd_vel', qos_subber)
    def ctrl_request_twist(self, twist_msg):
        self.velocity = twist_msg.linear.x
        self.steering = -twist_msg.angular.z

    def on_startup(self):
<<<<<<< HEAD
        self.velocity = 0.0
        self.steering = 0.0
        self.create_timer(self.DELTA_TIME, self.loop)

    def loop(self):
        self.actuation.send_control(self.steering, self.velocity)
<<<<<<< HEAD
=======
        self.localizer.get_state()
<<<<<<< HEAD
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
=======
=======

        self.velocity = 0.0
        self.steering = 0.0

        state = self.localizer.get_state()

        self.create_timer(self.DELTA_TIME, self.loop)

    def loop(self):
        state = self.localizer.get_state()
        
        self.actuation.send_control(self.steering, self.velocity)


>>>>>>> 9585fc8 (Teleop control in simulation with teleop_twist_keyboard added)
>>>>>>> 710b561 (Teleop control in simulation with teleop_twist_keyboard added)

if __name__ == '__main__':
    teleop_control.main()
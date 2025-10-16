#! /usr/bin/env python3

import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler

from svea_core.interfaces import LocalizationInterface
from svea_core.controllers.pure_pursuit import PurePursuitController
from svea_core.interfaces import ActuationInterface
from svea_core import rosonic as rx
from svea_core.utils import PlaceMarker, ShowPath


class pure_pursuit(rx.Node):  # Inherit from rx.Node

    r"""Pure Pursuit example script for SVEA.

    #**Background**

    This script implements a simple Pure Pursuit controller that follows a
    predefined path. The path is defined by a set of points, and the controller
    computes the steering angle and velocity to follow the path.

    The script also includes visualization of the goal and the path being
    followed.

    #**Preparation**

    TODO: Add instructions for setting up the teleoperation environment.

    #**Simulation**

    To run the Pure Pursuit example in simulation, you can use the following command:
    ```bash
    ros2 launch svea_examples floor2.xml is_sim:=true
    ```
    This launch file includes the following components, with example parameters:

        # Initial state of the robot (x, y, yaw, velocity)
        state:=[-7.4, -15.3, 0.9, 0.0] 
        # Points defining the path to follow. Each point is a string representation of a list.
        points:=['[-2.3,-7.1]','[10.5,11.7]','[5.7,15.0]','[-7.0,-4.0]'] 

    Attributes:
        points: List of points defining the path to follow.
        actuation: Actuation interface for sending control commands.
        localizer: Localization interface for receiving state information.
        mark: PlaceMarker for visualizing the goal.
        path: ShowPath for visualizing the path.
    """

    DELTA_TIME = 0.1
    TRAJ_LEN = 20

    points = rx.Parameter(['[-2.3, -7.1]', '[10.5, 11.7]', '[5.7, 15.0]', '[-7.0, -4.0]'])
    state = rx.Parameter([-7.4, -15.3, 0.9, 0.0])  # x, y, yaw, vel
    target_velocity = rx.Parameter(0.6)
    
    # Interfaces
    actuation = ActuationInterface()
    localizer = LocalizationInterface()
    # Goal Visualization
    mark = PlaceMarker()
    # Path Visualization
    path = ShowPath()

    def on_startup(self):
        """
        Initialize the Pure Pursuit controller and set up the path and goal.
        Controller is initialized with the target velocity and the points
        provided in the parameters. The current state is obtained from the
        localization interface, and the goal is set to the first point in the
        path.
        The trajectory is updated based on the current state and the goal.
        The controller is set to not finished initially, and a timer is created
        to call the loop method at regular intervals.
        """
        # Convert POINTS to numerical lists if loaded as strings
        if isinstance(self.points[0], str):
            self._points = [eval(point) for point in self.points]

        self.controller = PurePursuitController()
        self.controller.target_velocity = self.target_velocity

        state = self.localizer.get_state()
        x, y, yaw, vel = state

        self.curr = 0
        self.goal = self._points[self.curr]
        self.mark.marker('goal','blue',self.goal)
        self.update_traj(x, y)

        self.create_timer(self.DELTA_TIME, self.loop)

    def loop(self):
        """
        Main loop of the Pure Pursuit controller. It retrieves the current state
        from the localization interface, computes the steering and velocity
        commands using the controller, and sends these commands to the actuation
        interface.
        If the controller has finished following the path, it updates the goal
        and trajectory based on the next point in the path.
        """
        state = self.localizer.get_state()
        x, y, yaw, vel = state

        if self.controller.is_finished:
            self.update_goal()
            self.update_traj(x, y)

        steering, velocity = self.controller.compute_control(state)
        # self.get_logger().info(f"Steering: {steering}, Velocity: {velocity}")
        self.actuation.send_control(steering, velocity)

    def update_goal(self):
        """
        Update the goal to the next point in the path. If the end of the path
        is reached, it wraps around to the beginning. The current index is
        incremented, and the goal marker is updated.
        """
        self.curr += 1
        self.curr %= len(self._points)
        self.goal = self._points[self.curr]
        self.controller.is_finished = False
        # Mark the goal
        self.mark.marker('goal','blue',self.goal)

    def update_traj(self, x, y):
        """
        Update the trajectory based on the current state and the goal. It
        generates a linear trajectory from the current position to the goal
        position, and updates the controller's trajectory points.
        The trajectory is visualized using the ShowPath interface.
        """
        xs = np.linspace(x, self.goal[0], self.TRAJ_LEN)
        ys = np.linspace(y, self.goal[1], self.TRAJ_LEN)
        self.controller.traj_x = xs
        self.controller.traj_y = ys
        self.path.publish_path(xs,ys)

if __name__ == '__main__':
    pure_pursuit.main()
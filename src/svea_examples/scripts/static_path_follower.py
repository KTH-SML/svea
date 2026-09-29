#! /usr/bin/env python3

import numpy as np

from svea_core.interfaces import LocalizationInterface
from svea_core.controllers.pure_pursuit import PurePursuitController
from svea_core.interfaces import ActuationInterface, ShowMarker, ShowPath
from svea_core import rosonic as rx



class static_path_follower(rx.Node):

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
        goal_mark: ShowMarker for visualizing the goal.
        path: ShowPath for visualizing the path.
    """

    DELTA_TIME = 0.1
    TRAJ_LEN = 20

    points = rx.Parameter([1.0, 0.0, 0.62, 0.78, -0.22, 0.98, -0.9, 0.44, -0.9, -0.43, -0.22, -0.97, 0.62, -0.78, 1.0, -0.0])
    target_velocity = rx.Parameter(0.4)
    is_sim = rx.Parameter(True)
    
    # Interfaces
    
    actuation = ActuationInterface()
    localizer = LocalizationInterface()
    
    goal_marker = ShowMarker() # for goal visualization
    path = ShowPath() # for path visualization

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
        # Convert parameter to numerical list
        self._points = np.array(self.points).reshape(-1, 2).tolist()

        self.controller = PurePursuitController()
        self.controller.target_velocity = self.target_velocity
        self.controller.termination_distance = 0.5

        state = self.localizer.get_state()
        x, y, yaw, vel = state

        self.curr = 0
        self.goal = self._points[self.curr]
        self.update_traj(x, y)
        self.actuation.enable_difflock() 

        self.create_timer(self.DELTA_TIME, self.loop)
        
        def logger():
            self.goal_marker.place([*self.goal, 0.5], color='blue')
            self.path.publish_path(self.controller.traj_x,
                                   self.controller.traj_y)
        self.create_timer(1, logger)

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
        if self.is_sim:
            self.actuation.send_control(steering, velocity)
        else:
            self.actuation.send_control(steering, -1 * velocity)  # Invert velocity for real-world operation

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

if __name__ == '__main__':
    static_path_follower.main()

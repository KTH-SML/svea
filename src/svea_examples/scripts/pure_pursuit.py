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

    DELTA_TIME = 0.1
    TRAJ_LEN = 20

    points = rx.Parameter(['[-2.3, -7.1]', '[10.5, 11.7]', '[5.7, 15.0]', '[-7.0, -4.0]'])
    is_sim = rx.Parameter(True)
    target_velocity = rx.Parameter(1.0)

    actuation = ActuationInterface()
    localizer = LocalizationInterface()
    # Goal Visualization
    mark = PlaceMarker()
    # Path Visualization
    path = ShowPath()

    def on_startup(self):
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
        state = self.localizer.get_state()
        x, y, yaw, vel = state

        if self.controller.is_finished:
            self.update_goal()
            self.update_traj(x, y)

        steering, velocity = self.controller.compute_control(state)
        # self.get_logger().info(f"Steering: {steering}, Velocity: {velocity}")
        self.actuation.send_control(steering, velocity)

    def update_goal(self):
        self.curr += 1
        self.curr %= len(self._points)
        self.goal = self._points[self.curr]
        self.controller.is_finished = False
        # Mark the goal
        self.mark.marker('goal','blue',self.goal)

    def update_traj(self, x, y):
        xs = np.linspace(x, self.goal[0], self.TRAJ_LEN)
        ys = np.linspace(y, self.goal[1], self.TRAJ_LEN)
        self.controller.traj_x = xs
        self.controller.traj_y = ys
        self.path.publish_path(xs,ys)

if __name__ == '__main__':
    pure_pursuit.main()
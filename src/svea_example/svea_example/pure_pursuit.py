#! /usr/bin/env python3

import numpy as np

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler

from svea_core.interfaces import LocalizationInterface
from svea_core.controllers.pure_pursuit import PurePursuitController
from svea_core.interfaces import ActuationInterface
from svea_core import rosonic as rx


class pure_pursuit(rx.Node):  # Inherit from rx.Node

    DELTA_TIME = 0.01
    TRAJ_LEN = 10

    points = rx.Parameter(['[-2.3, -7.1]', '[10.5, 11.7]', '[5.7, 15.0]', '[-7.0, -4.0]'])
    is_sim = rx.Parameter(True)
    state = rx.Parameter([0.0, 0.0, 0.0, 0.0])
    target_velocity = rx.Parameter(1.0)

    def on_startup(self):
        # Convert POINTS to numerical lists if loaded as strings
        if isinstance(self.points[0], str):
            self._points = [eval(point) for point in self.points]

        self.curr = 0
        self.goal = self._points[self.curr]

        self.get_logger().info(f'Initial state: {self.state}')
        self.publish_initialpose(self.state)

        self.localizer = LocalizationInterface(self).start()
        self.actuation = ActuationInterface(self).start()
        self.controller = PurePursuitController()
        self.controller.target_velocity = self.target_velocity

        self.create_timer(1 / self.DELTA_TIME, self.spin)

    def spin(self):
        state = self.localizer.get_state()

        if self.controller.is_finished:
            self.update_goal()
            xs, ys = self.compute_traj(state[0], state[1])
            self.controller.tra_x = xs
            self.controller.tra_y = ys

        steering, velocity = self.controller.compute_control()
        self.actuation.send_control(steering, velocity)

    def update_goal(self):
        self.curr += 1
        self.curr %= len(self._points)
        self.goal = self._points[self.curr]
        self.controller.is_finished = False

    def compute_traj(self, x, y):
        xs = np.linspace(x, self.goal[0], self.TRAJ_LEN)
        ys = np.linspace(y, self.goal[1], self.TRAJ_LEN)
        return xs, ys

    def publish_initialpose(self, state, n=10):
        p = PoseWithCovarianceStamped()
        p.header.frame_id = 'map'
        p.pose.pose.position.x = state[0]
        p.pose.pose.position.y = state[1]

        q = quaternion_from_euler(0, 0, state[3])
        p.pose.pose.orientation.z = q[2]
        p.pose.pose.orientation.w = q[3]

        pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        rate = self.create_rate(0.1)

        for _ in range(n):
            pub.publish(p)


main = pure_pursuit.main


if __name__ == '__main__':
    main()
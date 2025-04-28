#! /usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler

from svea_core.models.bicycle import SimpleBicycleModel
from svea_core.interfaces import LocalizationInterface
from svea_core.controllers.pure_pursuit import PurePursuitController
from svea_core.interfaces import ActuationInterface
from svea_core.simulators.sim_SVEA import SimSVEA

def assert_points(pts):
    assert isinstance(pts, (list, tuple)), 'points is of wrong type, expected list'
    for xy in pts:
        assert isinstance(xy, (list, tuple)), 'points contain an element of wrong type, expected list of two values (x, y)'
        assert len(xy), 'points contain an element of wrong type, expected list of two values (x, y)'
        x, y = xy
        assert isinstance(x, (int, float)), 'points contain a coordinate pair wherein one value is not a number'
        assert isinstance(y, (int, float)), 'points contain a coordinate pair wherein one value is not a number'


class pure_pursuit(Node):  # Inherit from Node

    DELTA_TIME = 0.01
    TRAJ_LEN = 10
    TARGET_VELOCITY = 1.0
    RATE = 1e9

    def __init__(self):
        # Initialize the Node
        super().__init__('pure_pursuit')

        ## Parameters
        self.POINTS = self.load_param('~points', ['[-2.3, -7.1]', '[10.5, 11.7]', '[5.7, 15.0]', '[-7.0, -4.0]'])
        # Convert POINTS to numerical lists if loaded as strings
        if isinstance(self.POINTS[0], str):
            self.POINTS = [eval(point) for point in self.POINTS]
        self.IS_SIM = self.load_param('~is_sim', True)
        self.STATE = self.load_param('~state', [0.0, 0.0, 0.0, 0.0])

        assert_points(self.POINTS)

        ## Set initial values for node

        # initial state
        self.get_logger().info('VehicleState')
        self.get_logger().info(f'Initial state: {self.STATE}')
        self.publish_initialpose(self.STATE)

        # create goal state
        self.curr = 0
        self.goal = self.POINTS[self.curr]
        self.get_logger().info(f'Compute Trajectory to goal')

        if self.IS_SIM:

            # simulator need a model to simulate
            self.sim_model = SimpleBicycleModel()

            # start the simulator immediately, but paused
            self.simulator = SimSVEA(self.sim_model,
                                     dt=self.DELTA_TIME,
                                     run_lidar=True,
                                     start_paused=True).start()

        # create vehicle model
        self.localizer=LocalizationInterface(self).start()
        self.actuation=ActuationInterface(self).start()
        self.controller=PurePursuitController()
        self.controller.target_velocity = self.TARGET_VELOCITY

        if self.IS_SIM:
            self.simulator.toggle_pause_simulation()
       
    def load_param(self, name, value=None):
        self.declare_parameter(name, value)
        if value is None:
            assert self.has_parameter(name), f'Missing parameter "{name}"'
        else:
            self.get_logger().info(f'Parameter "{name}" set to {value}')
        return self.get_parameter(name).value

    def run(self):
        while self.keep_alive():
            self.spin()

    def keep_alive(self):
        return rclpy.ok()

    def spin(self):

        # limit the rate of main loop by waiting for state
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
        self.curr %= len(self.POINTS)
        self.goal = self.POINTS[self.curr]
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
            # rate.sleep()

def main(args=None):
    rclpy.init(args=args)

    # Initialize node
    node = pure_pursuit()

    # Spin the node
    try:
        rclpy.spin(node)
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down {}'.format(node.get_name()))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    


if __name__ == '__main__':
    main()
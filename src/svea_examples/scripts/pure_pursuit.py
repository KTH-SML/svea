#! /usr/bin/env python3

import numpy as np

import rospy
from rospy import Publisher, Rate
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

from svea.models.bicycle import SimpleBicycleModel
from svea.states import VehicleState
from svea.simulators.sim_SVEA import SimSVEA
from svea.interfaces import LocalizationInterface
from svea.controllers.pure_pursuit import PurePursuitController
from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.data import TrajDataHandler, RVIZPathHandler


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


def assert_points(pts):
    assert isinstance(pts, (list, tuple)), 'points is of wrong type, expected list'
    for xy in pts:
        assert isinstance(xy, (list, tuple)), 'points contain an element of wrong type, expected list of two values (x, y)'
        assert len(xy), 'points contain an element of wrong type, expected list of two values (x, y)'
        x, y = xy
        assert isinstance(x, (int, float)), 'points contain a coordinate pair wherein one value is not a number'
        assert isinstance(y, (int, float)), 'points contain a coordinate pair wherein one value is not a number'


def publish_initialpose(state, n=10):

    p = PoseWithCovarianceStamped()
    p.header.frame_id = 'map'
    p.pose.pose.position.x = state.x
    p.pose.pose.position.y = state.y

    q = quaternion_from_euler(0, 0, state.yaw)
    p.pose.pose.orientation.z = q[2]
    p.pose.pose.orientation.w = q[3]

    pub = Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rate = Rate(10)

    for _ in range(n):
        pub.publish(p)
        rate.sleep()


class pure_pursuit:

    DELTA_TIME = 0.01
    TRAJ_LEN = 10
    TARGET_VELOCITY = 1.0
    RATE = 1e9

    def __init__(self):

        ## Initialize node

        rospy.init_node('pure_pursuit')

        ## Parameters

        self.POINTS = load_param('~points')
        self.IS_SIM = load_param('~is_sim', False)
        self.USE_RVIZ = load_param('~use_rviz', False)
        self.STATE = load_param('~state', [0, 0, 0, 0])

        assert_points(self.POINTS)

        ## Set initial values for node

        # initial state
        state = VehicleState(*self.STATE)
        publish_initialpose(state)

        # create goal state
        self.curr = 0
        self.goal = self.POINTS[self.curr]
        xs, ys = self.compute_traj(state)

        ## Create simulators, models, managers, etc.

        if self.IS_SIM:

            # simulator need a model to simulate
            self.sim_model = SimpleBicycleModel(state)

            # start the simulator immediately, but paused
            self.simulator = SimSVEA(self.sim_model,
                                     dt=self.DELTA_TIME,
                                     run_lidar=True,
                                     start_paused=True).start()

        # start the SVEA manager
        self.svea = SVEAPurePursuit(LocalizationInterface,
                                    PurePursuitController,
                                    xs, ys,
                                    data_handler=RVIZPathHandler if self.USE_RVIZ else TrajDataHandler)

        self.svea.controller.target_velocity = self.TARGET_VELOCITY
        self.svea.start(wait=True)

        # everything ready to go -> unpause simulator
        if self.IS_SIM:
            self.simulator.toggle_pause_simulation()

    def run(self):
        while self.keep_alive():
            self.spin()

    def keep_alive(self):
        return not rospy.is_shutdown()

    def spin(self):

        # limit the rate of main loop by waiting for state
        state = self.svea.wait_for_state()

        if self.svea.is_finished:
            self.update_goal()
            xs, ys = self.compute_traj(state)
            self.svea.update_traj(xs, ys)

        steering, velocity = self.svea.compute_control()
        self.svea.send_control(steering, velocity)

        self.svea.visualize_data()

    def update_goal(self):
        self.curr += 1
        self.curr %= len(self.POINTS)
        self.goal = self.POINTS[self.curr]
        self.svea.controller.is_finished = False

    def compute_traj(self, state):
        xs = np.linspace(state.x, self.goal[0], self.TRAJ_LEN)
        ys = np.linspace(state.y, self.goal[1], self.TRAJ_LEN)
        return xs, ys


if __name__ == '__main__':

    ## Start node ##

    pure_pursuit().run()

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
from base_local_planner_controller import BaseLocalPlannerController
from svea.svea_managers.svea_archetypes import SVEAManager
from svea.data import TrajDataHandler, RVIZPathHandler


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


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


class main:

    DELTA_TIME = 0.01

    def __init__(self):

        ## Initialize node

        rospy.init_node('main')

        ## Parameters
        self.USE_RVIZ = load_param('~use_rviz', False)
        self.IS_SIM = load_param('~is_sim', False)
        self.STATE = load_param('~state', [1.79, 0, 0, 0])   # floor2: [-1.5, -0.7, 1, 0]
        ## Set initial values for node
        # initial state
        state = VehicleState(*self.STATE)
        publish_initialpose(state)

        ## Create simulators, models, managers, etc.
        if self.IS_SIM:

            # simulator need a model to simulate
            self.sim_model = SimpleBicycleModel(state)

            # start the simulator immediately, but paused
            self.simulator = SimSVEA(self.sim_model,
                                     dt=self.DELTA_TIME,
                                     run_lidar=True,
                                     start_paused=True,
                                     publish_odometry=True,
                                     publish_pose=True).start()

        # start the SVEA manager (needed for both sim and real world)
        self.svea = SVEAManager(LocalizationInterface,
                                    BaseLocalPlannerController,
                                    data_handler=RVIZPathHandler if self.USE_RVIZ else TrajDataHandler)
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

        state = self.svea.wait_for_state()                  # limit the rate of main loop by waiting for state
        steering, velocity = self.svea.compute_control()
        self.svea.send_control(steering, velocity)
        self.svea.visualize_data()


if __name__ == '__main__':
    ## Start node ##
    main().run()
#!/usr/bin/env python3

import math
import rospy
import numpy as np
import matplotlib.pyplot as plt

from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.states import VehicleState
from svea.interfaces import LocalizationInterface
from svea.controllers.pure_pursuit import PurePursuitController
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


class multi:

    DELTA_TIME = 0.1 # [s]
    VEH0_TARGET_VELOCITY = 0.3 # [m/s]
    VEH1_TARGET_VELOCITY = 0.8 # [m/s]

    def __init__(self):

        ## Initialize node

        rospy.init_node('multi')

        ## Parameters

        self.IS_SIM = load_param('~is_sim', True)
        self.VEH0_STATE = load_param('~veh0_state', [0, 0, 0, 0]) # [m, m, rad, m/s]
        self.VEH1_STATE = load_param('~veh1_state', [0, 0, 0, 0]) # [m, m, rad, m/s]
        self.ANIMATION = load_param('~animation', True)

        ## Set initial values for node

        self.veh0_name = 'veh0'
        self.veh0_state = VehicleState(*self.VEH0_STATE)

        self.veh1_name = 'veh1'
        self.veh1_state = VehicleState(*self.VEH1_STATE)

        # Trajectory
        traj_xs = np.arange(0, 5, 0.1)
        traj_ys = [np.sin(x) * x for x in traj_xs]

        ## Create simulators, models, managers, etc.

        if self.IS_SIM:

            # simulators need a model to simulate

            self.veh0_sim_model = SimpleBicycleModel(self.veh0_state)

            self.veh1_sim_model = SimpleBicycleModel(self.veh1_state)

            # start the simulators immediately, but paused

            self.veh0_simulator = SimSVEA(self.veh0_sim_model,
                                          vehicle_name=self.veh0_name,
                                          dt=self.DELTA_TIME,
                                          run_lidar=True,
                                          start_paused=True).start()

            self.veh1_simulator = SimSVEA(self.veh1_sim_model,
                                          vehicle_name=self.veh1_name,
                                          dt=self.DELTA_TIME,
                                          run_lidar=True,
                                          start_paused=True).start()

        # start the SVEA managers

        self.veh0_mgr = SVEAPurePursuit(LocalizationInterface,
                                        PurePursuitController,
                                        traj_xs, traj_ys,
                                        vehicle_name=self.veh0_name)

        self.veh1_mgr = SVEAPurePursuit(LocalizationInterface,
                                        PurePursuitController,
                                        traj_xs, traj_ys,
                                        vehicle_name=self.veh1_name)

        self.veh0_mgr.controller.target_velocity = self.VEH0_TARGET_VELOCITY
        self.veh1_mgr.controller.target_velocity = self.VEH1_TARGET_VELOCITY

        self.veh0_mgr.start(wait=True)
        self.veh1_mgr.start(wait=True)

        # everything ready to go -> unpause simulator
        if self.IS_SIM:
            self.veh0_simulator.toggle_pause_simulation()
            self.veh1_simulator.toggle_pause_simulation()

        # wait for both states to load before starting
        self.veh0_mgr.wait_for_state()
        self.veh1_mgr.wait_for_state()

    def run(self):
        while self.keep_alive():
            self.spin()

    def keep_alive(self):
        veh0_fin = self.veh0_mgr.is_finished
        veh1_fin = self.veh1_mgr.is_finished
        ros_fin = rospy.is_shutdown()
        return not any([veh0_fin, veh1_fin, ros_fin])

    def spin(self):

        self.veh0_mgr.wait_for_state()

        # compute control input via pure pursuit
        steering, velocity = self.veh0_mgr.compute_control()
        self.veh0_mgr.send_control(steering, velocity)

        steering, velocity = self.veh1_mgr.compute_control()
        self.veh1_mgr.send_control(steering, velocity)

        # visualize data
        if self.ANIMATION:
            plt.cla()
            self.veh0_mgr.data_handler.visualize_data(only_plot = False)
            self.veh1_mgr.data_handler.visualize_data(only_plot = False)
            plt.pause(0.001)

if __name__ == '__main__':

    ## Start node ##

    multi().run()


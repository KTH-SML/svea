#!/usr/bin/env python

import math
import rospy
import numpy as np
import matplotlib.pyplot as plt

from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.states import VehicleState
from svea.localizers import LocalizationInterface
from svea.controllers.pure_pursuit import PurePursuitController
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA

## SIMULATION PARAMS ##########################################################
vehicle_name0 = "SVEA0"
init_state0 = [0, 0, 0, 0] # [x, y, yaw, v], units: [m, m, rad, m/s]
init_state0 = VehicleState(*init_state0)
target_velocity0 = 0.3

vehicle_name1 = "SVEA1"
init_state1 = [-2, 0, 0, 0] # [x, y, yaw, v], units: [m, m, rad, m/s]
init_state1 = VehicleState(*init_state1)
target_velocity1 = 0.8 # [m/s]

dt = 0.01

# trajectory
traj_x = np.arange(0, 5, 0.1)
traj_y = [math.sin(ix) * ix for ix in traj_x]

# animate results?
show_animation = True
###############################################################################

def param_init():
    # grab parameters from launch-file
    is_sim_param = rospy.search_param('is_sim')
    is_sim = rospy.get_param(is_sim_param, True)
    return is_sim

def main():
    rospy.init_node('SVEA_multi')
    is_sim = param_init()

    if is_sim:
        # start the simulation
        model_for_sim0 = SimpleBicycleModel(init_state0)
        simulator0 = SimSVEA(vehicle_name0, model_for_sim0,
                             dt=dt, run_lidar=True, start_paused=True).start()
        model_for_sim1 = SimpleBicycleModel(init_state1)
        simulator1 = SimSVEA(vehicle_name1, model_for_sim1,
                             dt=dt, run_lidar=True, start_paused=True).start()

    # initialize pure pursuit SVEA managers
    svea0 = SVEAPurePursuit(vehicle_name0,
                            LocalizationInterface,
                            PurePursuitController,
                            traj_x, traj_y)
    svea1 = SVEAPurePursuit(vehicle_name1,
                            LocalizationInterface,
                            PurePursuitController,
                            traj_x, traj_y)
    svea0.start(wait=True)
    svea1.start(wait=True)

    if is_sim:
        # start simulation
        simulator0.toggle_pause_simulation()
        simulator1.toggle_pause_simulation()

    # simualtion + animation loop
    svea0.controller.target_velocity = target_velocity0
    svea1.controller.target_velocity = target_velocity1

    # wait for both states to load
    svea0.wait_for_state()
    svea1.wait_for_state()
    while not svea0.is_finished and not svea1.is_finished \
          and not rospy.is_shutdown():

        svea0.wait_for_state() # just time based on svea0 state

        # compute control input via pure pursuit
        steering, velocity = svea0.compute_control()
        svea0.send_control(steering, velocity)

        steering, velocity = svea1.compute_control()
        svea1.send_control(steering, velocity)

        # visualize data
        if show_animation:
            plt.cla()
            svea0.data_handler.visualize_data(only_plot = False)
            svea1.data_handler.visualize_data(only_plot = False)
            plt.pause(0.001)

    rospy.spin()

if __name__ == '__main__':
    main()

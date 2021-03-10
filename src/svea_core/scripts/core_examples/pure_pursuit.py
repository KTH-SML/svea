#!/usr/bin/env python

import math
import rospy
import numpy as np

from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.states import VehicleState
from svea.localizers import LocalizationInterface
from svea.controllers.pure_pursuit import PurePursuitController
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA


## SIMULATION PARAMS ##########################################################
init_state = [0.0, 0.0, 0.0, 0.0] #[x, y, yaw, v], units: [m, m, rad, m/s]
init_state = VehicleState(*init_state)
target_velocity = 0.6 # [m/s]
dt = 0.01

# trajectory
traj_x = np.arange(0, 5, 0.1)
traj_y = [math.sin(ix) * ix for ix in traj_x]

show_animation = True
###############################################################################


def param_init():
    # grab parameters from launch-file
    is_sim_param = rospy.search_param('is_sim')
    is_sim = rospy.get_param(is_sim_param, True)
    return is_sim

def main():
    rospy.init_node('SVEA_purepursuit')
    is_sim = param_init()

    if is_sim:
        # start the simulation
        model_for_sim = SimpleBicycleModel(init_state)
        simulator = SimSVEA(model_for_sim, dt=dt, start_paused=True).start()

    # start pure pursuit SVEA manager
    svea = SVEAPurePursuit(LocalizationInterface,
                           PurePursuitController,
                           traj_x, traj_y)
    svea.start(wait=True)

    if is_sim:
        # start simulation
        simulator.toggle_pause_simulation()

    # simualtion loop
    svea.controller.target_velocity = target_velocity
    while not svea.is_finished and not rospy.is_shutdown():
        state = svea.wait_for_state()

        # compute control input via pure pursuit
        steering, velocity = svea.compute_control(state)
        svea.send_control(steering, velocity)

        # visualize data
        if show_animation:
            svea.visualize_data()
        else:
            rospy.loginfo_throttle(1, state)

    rospy.loginfo("Trajectory finished!")

    rospy.spin()

if __name__ == '__main__':
    main()

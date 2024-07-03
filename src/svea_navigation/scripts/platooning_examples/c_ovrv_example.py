#!/usr/bin/env python

import math
import rospy
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from svea.states import VehicleState
from svea.simulators.sim_SVEA import SimSVEA
from svea.models.bicycle import SimpleBicycleModel
from svea.svea_managers.path_following_sveas import SVEAPlatoonMember
from svea.interfaces import LocalizationInterface
from svea.data import RVIZPathHandler
from svea.models.cooperative import C_OVRV
from .c_ovrv_utils import *

mpl.style.use('seaborn')

## C-OVRV PARAMS ##############################################################
platoon_size = 5
num_neighbors = 2 # 0 for don't use communicated info
desired_time_headway = 0.3
k1 = 0.5
k2 = 0.5
k3 = 0.8
k4 = 0.8
k_gains = [k1, k2, k3, k4]
min_spacing = 0.3
dt = 0.01
###############################################################################

## EXPERIMENT SET UP ##########################################################
init_spacing = 0.5  # initial space between bumpers
init_velocity = 1.2  # initial target velocity
disturbance_velocity = 0.6 # experiment velocity drop

steady_state_hold_time = 12.0  # seconds

# trajectory
xs = [-2.05, 14.8]
ys = [-6.87, 18.2]
traj_x = np.linspace(xs[0], xs[1]).tolist()
traj_y = np.linspace(ys[0], ys[1]).tolist()
###############################################################################

## SVEA #######################################################################
leader_name = "SVEA0"
follower_prefix = "SVEA"
###############################################################################

## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
visualize_plot = True
###############################################################################


def param_init():
    """Initialization handles use with just python or in a launch file
    """
    # grab parameters from launch-file
    start_pt_param = rospy.search_param('start_pt')
    use_rviz_param = rospy.search_param('use_rviz')

    start_pt = rospy.get_param(start_pt_param, default_init_pt)
    if isinstance(start_pt, str):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]
    use_rviz = rospy.get_param(use_rviz_param, False)

    return start_pt, use_rviz


def main():
    rospy.init_node('c_ovrv_example')
    last_vehicle_start_pt, use_rviz = param_init()

    # compute initial positions, these correspond with initial SVEA placement
    init_spacings = [init_spacing for _ in range(platoon_size)]
    leader_start_pt, follower_start_pts = \
        compute_positions_from_spacings(last_vehicle_start_pt, init_spacings)

    ## For testing ############################################################
    c_ovrv_model = C_OVRV(platoon_size, num_neighbors, k_gains, min_spacing,
                          desired_time_headway, init_velocity, dt=dt)
    init_eq_pt = c_ovrv_model.equilibrium_pt
    leader_start_pt, follower_start_pts = \
        compute_positions_from_spacings(follower_start_pts[-1],
                                        init_eq_pt[:platoon_size])
    ###########################################################################

    # create simulated leader and followers
    leader_state = VehicleState(*leader_start_pt)
    leader_sim = SimSVEA(SimpleBicycleModel(leader_state),
                         vehicle_name=leader_name,
                         dt=dt, start_paused=True).start()
    leader = SVEAPlatoonMember(LocalizationInterface,
                               traj_x, traj_y,
                               data_handler = RVIZPathHandler,
                               vehicle_name = leader_name)

    follower_sims = []
    followers = []
    for i in range(platoon_size):
        follower_name = follower_prefix + str(1 + i)
        follower_state = VehicleState(*follower_start_pts[i])
        follower_sim = SimSVEA(SimpleBicycleModel(follower_state),
                               vehicle_name = follower_name,
                               dt=dt, start_paused=True).start()
        follower = SVEAPlatoonMember(LocalizationInterface,
                                     traj_x, traj_y,
                                     data_handler = RVIZPathHandler,
                                     vehicle_name = follower_name)
        follower_sims.append(follower_sim)
        followers.append(follower)

    # spin up svea managers so they are ready before simulation unpauses
    leader.start(wait=True)
    [follower.start(wait=True) for follower in followers]

    # unpause the simulated vehicles
    toggle_pause(leader_sim, follower_sims)
    wait_for_platoon_states(leader, followers)

    if use_rviz:
        leader.data_handler.pub_car_poly()
        [follower.data_handler.pub_car_poly() for follower in followers]

    # create cooperative model
    c_ovrv_model = C_OVRV(platoon_size, num_neighbors, k_gains, min_spacing,
                          desired_time_headway, init_velocity, dt=dt)
    init_eq_pt = c_ovrv_model.equilibrium_pt
    print(("the initial eq points", init_eq_pt))
    leader_eq_pt, follower_eq_pts = \
        compute_positions_from_spacings(follower_start_pts[-1],
                                        init_eq_pt[:platoon_size])

    # get each vehicle into close-to-equilibrium positions
    rospy.loginfo("Going to initial equilibrium positions")
    goto_eq_positions(leader, leader_eq_pt, followers, follower_eq_pts)

    # create unified data logs for platoon
    start_t = rospy.get_time()
    platoon_t = []
    leader_v = []
    follower_vs = [[] for follower in followers]

    # simualtion + animation loop
    experiment_begun = False
    reaching_speed = True
    prev_t = start_t
    timer = steady_state_hold_time
    experiment_start_time = -float('inf')
    while not leader.is_finished and not rospy.is_shutdown():
        # update all vehicle states
        leader_state = leader.wait_for_state()
        follower_states = [follower.wait_for_state() for follower in followers]
        curr_t = rospy.get_time() - start_t

        platoon_t.append(curr_t)
        leader_v.append(leader_state.v)
        [follower_vs[i].append(follower_state.v)
         for i, follower_state in enumerate(follower_states)]

        if not experiment_begun and reaching_speed:
            # use velocity control until reached steady state
            rospy.loginfo_once("Reaching steady state speeds")
            leader.send_vel(init_velocity)
            [follower.send_vel(init_velocity) for follower in followers]
            # keep track of latest time
            experiment_start_time = max(experiment_start_time, curr_t)
            if not reached_steady_state(init_velocity, leader, followers):
                timer = steady_state_hold_time
            else:
                timer -= curr_t - prev_t
            prev_t = curr_t
            reaching_speed = timer > 0.0
        else:
            rospy.loginfo_once("Beginning Experiment at t="+str(curr_t))
            experiment_begun = True

            # compute accelerations
            spacings = compute_spacings(leader, followers)
            speeds = [follower_state.v for follower_state in follower_states]
            accel_ctrls = c_ovrv_model.compute_accel(spacings, speeds,
                                                     leader_state.v)

            # creating slow down for leader
            leader.send_vel(disturbance_velocity)

            for i, follower in enumerate(followers):
                if spacings[i] > min_spacing:
                    follower.send_accel(accel_ctrls[i], dt)
                else:
                    follower.send_vel(0.0)

        # update visualizations
        if use_rviz:
            # vizualize leader's car poly, path, and target pt
            leader.visualize_data()
            [follower.data_handler.pub_car_poly() for follower in followers]

    if not rospy.is_shutdown():
        rospy.loginfo("Trajectory finished.")

    if visualize_plot:
        plt.plot(platoon_t, leader_v, "-", linewidth=1, label="V_L")
        [plt.plot(platoon_t, follower_v, "-", linewidth=1, label="V_" + str(i))
            for i, follower_v in enumerate(follower_vs)]
        plt.axvline(experiment_start_time, 0, 1,
                    linestyle="--", color="grey", alpha=0.5)
        plt.xlabel('time (s)')
        plt.ylabel('velocity (m/s)')
        plt.ylim(-0.2, 1.8)
        plt.legend(loc="upper right")
        plt.title(str(platoon_size) +
                  " vehicle platoon with k = " + str(num_neighbors))
        plt.show()
        plt.pause(0.001)

    rospy.spin()


if __name__ == '__main__':
    main()

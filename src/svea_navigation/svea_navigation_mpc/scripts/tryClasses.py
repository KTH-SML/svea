#! /usr/bin/env python3
from mpc_casadi import MPC
import numpy as np

if __name__ == '__main__':
    
    config_path = '/svea_ws/src/svea_navigation/svea_navigation_mpc/params/mpc_params.yaml'
    mpc_controller = MPC(config_path)

    initial_state = [5,5,0.6,0.0]
    reference_state = [6,5,0.6,0.5]

    x_ref = np.tile(reference_state, (26, 1)).T 

    steer, acc = mpc_controller.compute_control(initial_state,x_ref)

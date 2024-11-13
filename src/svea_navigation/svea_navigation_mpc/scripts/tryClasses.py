#! /usr/bin/env python3
from svea.controllers.mpc import MPC_casadi
import numpy as np

if __name__ == '__main__':
    
    mpc_controller = MPC_casadi()

    initial_state = [5,5,0,0.0]
    reference_state = [10,5,0.0,0.5]

    x_ref = np.tile(reference_state, (26, 1)).T 

    steer, velocity = mpc_controller.compute_control(initial_state,x_ref)

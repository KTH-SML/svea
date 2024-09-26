#! /usr/bin/env python3
import casadi as ca
import numpy as np
import yaml

class MPC:
    def __init__(self, config_path):
        """
        Initialize the MPC controller with the given parameters.

        :param N: Prediction horizon steps
        :param dt: Sampling time
        :param min_steering: Minimum steering angle [rad]
        :param max_steering: Maximum steering angle [rad]
        :param min_acceleration: Minimum acceleration [m/s^2]
        :param max_acceleration: Maximum acceleration [m/s^2]
        :param Q1: State weight matrix (4x4)
        :param Q2: Control weight matrix (2x2)
        :param Qf: Final state weight matrix (4x4)
        """
        # Load parameters from the YAML file 
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        # Initialize parameters from YAML file
        self.N = config['prediction_horizon']  # Prediction horizon
        self.dt = ca.DM(config['time_step'])  # Sampling time
        self.L = ca.DM(config['base_length'])  # Wheelbase of the vehicle
        
        # Load state weight matrix Q1, control weight matrix Q2, and final state weight matrix Qf
        Q1_list = config['state_weight_matrix']
        self.Q1 = ca.DM(np.array(Q1_list).reshape((4, 4)))    # conversion to dense matrix to allow symbolic operations.

        Q2_list = config['control_weight_matrix']
        self.Q2 = ca.DM(np.array(Q2_list).reshape((2, 2)))

        Qf_list = config['final_state_weight_matrix']
        self.Qf = ca.DM(np.array(Qf_list).reshape((4, 4)))
        
        # Load input constraints from the YAML file
        self.min_steering = np.radians(config['steering_min'])  # Steering angle [rad]
        self.max_steering = np.radians(config['steering_max'])  # Steering angle [rad]
        self.min_acceleration = config['acceleration_min']  # Acceleration [m/s^2]
        self.max_acceleration = config['acceleration_max']  # Acceleration [m/s^2]
        
        self.opti = ca.Opti()  # CasADi optimization problem

        # Define state and control variables
        self.x = self.opti.variable(4, self.N + 1)  # state = [x, y, theta, v]
        self.u = self.opti.variable(2, self.N)  # input = [steering, acceleration]

        self.x_init = self.opti.parameter(4)  # Initial state
        self.x_ref = self.opti.parameter(4, self.N + 1)  # Reference trajectory

        # Define the objective function: J = (x[k]-x_ref[k])^T Q1 (x[k]-x_ref[k]) + (u[k+1] - u[k])^T Q2 (u[k+1] - u[k])
        self.objective = 0
        for k in range(self.N):
            state_error = self.x[:, k] - self.x_ref[:, k]
            if k < self.N-1: 
                input_cost = self.u[:,k+1] - self.u[:,k]
            else:
                input_cost = ca.DM.zeros(self.u.shape[0], 1)
            self.objective += ca.mtimes([state_error.T, self.Q1, state_error]) + \
                              ca.mtimes([input_cost.T, self.Q2, input_cost])
        # Final state cost
        final_state_error = self.x[:, self.N] - self.x_ref[:, self.N]
        self.objective += ca.mtimes([final_state_error.T, self.Qf, final_state_error])

        # Vehicle dynamics constraints
        for k in range(self.N):
            x_next = self.x[0, k] + self.dt * self.x[3, k] * ca.cos(self.x[2, k])
            y_next = self.x[1, k] + self.dt * self.x[3, k] * ca.sin(self.x[2, k])
            theta_next = self.x[2, k] + self.dt * (self.x[3, k] / self.L) * ca.tan(self.u[0, k])
            v_next = self.x[3, k] + self.dt * self.u[1, k]

            self.opti.subject_to(self.x[0, k + 1] == x_next)
            self.opti.subject_to(self.x[1, k + 1] == y_next)
            self.opti.subject_to(self.x[2, k + 1] == theta_next)
            self.opti.subject_to(self.x[3, k + 1] == v_next)

        # Input constraints (steering, acceleration)
        for k in range(self.N):
            self.opti.subject_to(self.min_steering <= self.u[0, k])
            self.opti.subject_to(self.u[0, k] <= self.max_steering)
            self.opti.subject_to(self.min_acceleration <= self.u[1, k])
            self.opti.subject_to(self.u[1, k] <= self.max_acceleration)

        # Initial state constraint
        self.opti.subject_to(self.x[:, 0] == self.x_init)

        # Set solver options
        opts = {"ipopt.print_level": 0, "print_time": 0}
        self.opti.solver("ipopt", opts)

    def compute_control(self, state, reference_trajectory):
        """
        Compute the control actions (steering, acceleration) using MPC.

        :param state: Current state of the vehicle [x, y, theta, v]
        :param reference_trajectory: Reference trajectory [4, N+1]
        :return: steering, acceleration
        """
        # Set current state and reference trajectory
        self.opti.set_value(self.x_init, state)
        self.opti.set_value(self.x_ref, reference_trajectory)

        # Set initial guess for controls with the correct dimensions
        initial_control_guess = np.array([[0.0] * self.N, [0.1] * self.N])  
        #self.opti.set_initial(self.u, initial_control_guess)

        # Solve the optimization problem
        sol = self.opti.solve()

        # Extract control actions
        steering = sol.value(self.u[0, 0])
        acceleration = sol.value(self.u[1, 0])

        print(sol.value(self.x))
        return steering, acceleration
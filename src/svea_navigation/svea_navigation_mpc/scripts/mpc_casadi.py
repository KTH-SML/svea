#! /usr/bin/env python3
import casadi as ca
import numpy as np
import yaml

class MPC_casadi:
    def __init__(self,vehicle_name=''):
        """
        Initialize the MPC controller with the given parameters.

        :param N: Prediction horizon steps
        :param dt: Sampling time
        :param min_steering: Minimum steering angle [rad]
        :param max_steering: Maximum steering angle [rad]
        :param min_acceleration: Minimum acceleration [m/s^2]
        :param max_acceleration: Maximum acceleration [m/s^2]
        :param min_velocity: Minimum velovity [m/s]
        :param max_velocity: Maximum velocity [m/s]
        :param Q1: State weight matrix (4x4)
        :param Q2: Control rate weight matrix (2x2)
        :param Q3: Control weight matrix (2x2)
        :param Qf: Final state weight matrix (4x4)
        """
        config_path = '/svea_ws/src/svea_navigation/svea_navigation_mpc/params/mpc_params.yaml'
        # Load parameters from the YAML file 
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        # Initialize parameters from YAML file
        self.N = config['prediction_horizon']  # maximum prediction horizon
        self.current_horizon = self.N          # Initially set to max horizon
        self.dt = ca.DM(config['time_step'])  # Sampling time
        self.L = ca.DM(config['base_length'])  # Wheelbase of the vehicle
        
        # Load state weight matrix Q1, control weight matrix Q2, and final state weight matrix Qf
        Q1_list = config['state_weight_matrix']
        self.Q1 = ca.DM(np.array(Q1_list).reshape((4, 4)))    # conversion to dense matrix to allow symbolic operations.

        Q2_list = config['control_rate_weight_matrix']
        self.Q2 = ca.DM(np.array(Q2_list).reshape((2, 2)))

        Q3_list = config['control_weight_matrix']
        self.Q3 = ca.DM(np.array(Q3_list).reshape((2, 2)))

        Qf_list = config['final_state_weight_matrix']
        self.Qf = ca.DM(np.array(Qf_list).reshape((4, 4)))

        self.Qv = ca.DM(config['velocity_deadzone_weight_matrix'])
        
        # Load constraints from the YAML file
        self.min_steering = np.radians(config['steering_min'])  
        self.max_steering = np.radians(config['steering_max']) 
        self.min_acceleration = config['acceleration_min']  
        self.max_acceleration = config['acceleration_max']  
        self.min_velocity = config['velocity_min']  
        self.max_velocity = config['velocity_max']  
        self.min_steering_rate = np.radians(config['steering_rate_min'])  
        self.max_steering_rate = np.radians(config['steering_rate_max'])
        self.velocity_deadzone = config['velocity_deadzone'] 
        
        self.opti = ca.Opti()  # CasADi optimization problem

        self.define_state_and_control_variables()
        self.set_objective_function()
        self.set_state_constraints()
        self.set_control_input_constraints()
        self.set_solver_options()

    def compute_control(self, state, reference_trajectory):
        """
        Compute the control actions (steering, acceleration) using MPC.
        
        :param state: Current state of the vehicle [x, y, theta, v, delta]
        :param reference_trajectory: Reference trajectory [4, N+1]
        :return: steering, velocity
        """
        # Bound the initial state to respect constraints
        bounded_state = self.bound_initial_state(state)

        # Enlarge the reference trajectory with a fictitious velocity and steering value for feasibility.
        # Add a row of zeros as the last row to the reference_trajectory matrix.
        reference_trajectory = ca.vertcat(reference_trajectory, ca.DM.zeros(2, reference_trajectory.shape[1]))

        # Set current state and reference trajectory for the active part of the horizon
        self.opti.set_value(self.x_init, bounded_state)
        self.opti.set_value(self.x_ref[:, :self.current_horizon+1], reference_trajectory[:, :self.current_horizon+1])

        # Solve the optimization problem
        self.sol = self.opti.solve()

        # Extract control actions (acceleration and steering rate)
        acceleration = self.sol.value(self.u[0, 0])
        steering_rate = self.sol.value(self.u[1, 0])

        return steering_rate, acceleration
    
    def set_new_prediction_horizon(self, new_horizon):
        """
        Dynamically adjust the active horizon for the optimization problem.
        When the horizon is reduced, you simply freeze unused variables and update the reference trajectory 
        and state for the active part of the horizon. The solver will then only optimize over the active steps.
        """
        self.current_horizon = new_horizon
        
        # Redefine objective function with new horizon.
        self.set_objective_function()

    def get_optimal_control(self, all=True):
        """
        This method returns the optimal control computed by the mpc.
        :param all: Flag to indicate if all of the optimal control for the 
            whole prediction should	be returned or only the first step.
        :type all: bool
        :return: The optimal control computed by the mpc controller.
        :rtype: NumPy array
        """
        if self.sol is not None:
            u_opt = np.array(self.sol.value(self.u))
            if all:
                return u_opt
            else:
                return u_opt[:,0]
        else:
            return None
        
    def get_optimal_states(self):
        """
        This method returns the optimal states computed by the mpc controller
        for the whole prediction horizon.
        :return: The optimal states for the whole prediction horizon.
        :rtype: NumPy array
        """
        if self.sol is not None:
            return np.array(self.sol.value(self.x))   # to get only optimized one: self.sol.value(self.x[:, :self.current_horizon])
        else: 
            return None

    def define_state_and_control_variables(self):
        # Define state and control variables
        self.x = self.opti.variable(5, self.N + 1)  # state = [x, y, theta, v]
        self.u = self.opti.variable(2, self.N)      # input = [steering, acceleration]

        self.x_init = self.opti.parameter(5)             # Initial state
        self.x_ref = self.opti.parameter(5, self.N + 1)  # Reference trajectory

    def set_objective_function(self):
        # Define the objective function: 
        # J = (x[k]-x_ref[k])^T Q1 (x[k]-x_ref[k]) + (u[k+1] - u[k])^T Q2 (u[k+1] - u[k]) + u[k]^T Q3 u[k]
        self.objective = 0
        for k in range(self.current_horizon):
            # State error term (ignore delta in reference trajectory)
            state_error = self.x[0:4, k] - self.x_ref[0:4, k]

            # Control input rate of change term (u[k+1] - u[k])
            if k < self.current_horizon - 1:
                input_cost = self.u[:, k+1] - self.u[:, k]
            else:
                input_cost = ca.DM.zeros(self.u.shape[0], 1)

            # Penalize for violating the velocity deadzone (soft constraint)
            velocity_error = ca.fmax(0, self.velocity_deadzone - ca.fabs(self.x[3, k]))  # Penalize if |v| < velocity_deadzone

            # Accumulate the terms into the objective
            self.objective += ca.mtimes([state_error.T, self.Q1, state_error]) + \
                            ca.mtimes([input_cost.T, self.Q2, input_cost]) + \
                            ca.mtimes([self.u[:, k].T, self.Q3, self.u[:, k]]) + \
                            ca.mtimes([velocity_error.T, self.Qv, velocity_error])

        # Final state cost
        final_state_error = self.x[0:4, self.current_horizon] - self.x_ref[0:4, self.current_horizon]
        self.objective += ca.mtimes([final_state_error.T, self.Qf, final_state_error])

        # Specify type of optimization problem
        self.opti.minimize(self.objective)

    def set_state_constraints(self):
        # Initial state constraint
        self.opti.subject_to(self.x[:, 0] == self.x_init)

        # Vehicle dynamics constraints
        for k in range(self.N):
            x_next = self.x[0, k] + self.dt * self.x[3, k] * ca.cos(self.x[2, k])  # x_k+1 = x_k + dt * v_k * cos(theta_k)
            y_next = self.x[1, k] + self.dt * self.x[3, k] * ca.sin(self.x[2, k])  # y_k+1 = y_k + dt * v_k * sin(theta_k)
            theta_next = self.x[2, k] + self.dt * (self.x[3, k] / self.L) * ca.tan(self.x[4, k])  # theta_k+1 = theta_k + dt * v_k * tan(delta_k) / L
            v_next = self.x[3, k] + self.dt * self.u[0, k]  # v_k+1 = v_k + dt * a_k
            delta_next = self.x[4, k] + self.dt * self.u[1, k]  # delta_k+1 = delta_k + dt * steering_rate_k

            self.opti.subject_to(self.x[0, k + 1] == x_next)
            self.opti.subject_to(self.x[1, k + 1] == y_next)
            self.opti.subject_to(self.x[2, k + 1] == theta_next)
            self.opti.subject_to(self.x[3, k + 1] == v_next)
            self.opti.subject_to(self.x[4, k + 1] == delta_next)

            # Velocity constraints
            self.opti.subject_to(self.x[3, k] <= self.max_velocity)
            self.opti.subject_to(self.x[3, k] >= self.min_velocity)

            # Steering angle constraints
            self.opti.subject_to(self.x[4, k] <= self.max_steering)
            self.opti.subject_to(self.x[4, k] >= self.min_steering)
    
    def set_control_input_constraints(self):
        # Input constraints (acceleration, steering rate)
        for k in range(self.N):
            self.opti.subject_to(self.min_acceleration <= self.u[0, k])
            self.opti.subject_to(self.u[0, k] <= self.max_acceleration)

            # Steering rate constraint
            self.opti.subject_to(self.min_steering_rate <= self.u[1, k])
            self.opti.subject_to(self.u[1, k] <= self.max_steering_rate)

    def set_solver_options(self):
        # Set solver options
        opts = {"ipopt.print_level": 0, "print_time": 0}
        self.opti.solver("ipopt", opts)

    def bound_initial_state(self,state):
        """
        This method checks if the initial state provided to the mpc, which comes from the localization stack,
        is within the allowed bounds. If not, it clamps it.
        """
        # Ensure the velocity is within the specified bounds
        clamped_velocity = max(self.min_velocity, min(state[3], self.max_velocity))
        
        # Create a new state with the clamped velocity
        bounded_state = state.copy()
        bounded_state[3] = clamped_velocity
        
        return bounded_state
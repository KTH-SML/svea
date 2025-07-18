#! /usr/bin/env python3

import casadi as ca
import numpy as np
from rclpy.node import Node

def load_param(self, name, value=None):
        self._node.declare_parameter(name, value)
        if value is None:
            assert self._node.has_parameter(name), f'Missing parameter "{name}"'
        return self._node.get_parameter(name).value

class MPC_casadi:

    def __init__(self, node: Node, config_ns='/mpc') -> None:
        """
        This is the release 0 of a general-purpose Nonlinear Model Predictive Controller (NMPC) 
        designed for the SVEA platform. It offers the following features:

        - Nonlinear dynamics with state representation [x, y, theta, v, steering] 
        and control inputs [steering_rate, acceleration].
        - Accepts a reference trajectory as input (see `compute_control` method) 
        and optimizes the predicted trajectory to minimize the deviation of 
        the predicted points from the next reference point.
        - Functions as both an optimal path planner and path tracker:
            - For path planning and tracking: Reference points should be spaced apart to give 
                the MPC the freedom to determine the optimal path.
            - For tracking only: Adjust parameters such as prediction horizon to 
                follow the reference trajectory closely.

        Limitations:
        - Neither static nor dynamic obstacles are considered 
        in this implementation.

        Initialize the MPC controller with the given parameters:

        :param L: Wheelbase of the vehicle (unit [m])
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
        :param Qv: Forward_speed_weight scalar
        """

        ## Core Parameters

        self._node = node

        # The prediction horizon steps for the mpc optimization problem.
        self.N = load_param(f'{config_ns}/prediction_horizon')
        self.current_horizon = self.N # Initially set to max horizon

        # The time step in which the optimization problem is divided (unit [s]).
        self.dt = ca.DM(load_param(f'{config_ns}/time_step'))
        
        ## Load weight matrices 
        # Note: we convert to dense matrix to allow symbolic operations.

        self.Q1_list = load_param(f'{config_ns}/state_weight_matrix')
        self.Q1 = ca.DM(np.array(self.Q1_list).reshape((4, 4)))

        self.Q2_list = load_param(f'{config_ns}/control_rate_weight_matrix')
        self.Q2 = ca.DM(np.array(self.Q2_list).reshape((2, 2)))

        self.Q3_list = load_param(f'{config_ns}/control_weight_matrix')
        self.Q3 = ca.DM(np.array(self.Q3_list).reshape((2, 2)))

        self.Qf_list = load_param(f'{config_ns}/final_state_weight_matrix')
        self.Qf = ca.DM(np.array(self.Qf_list).reshape((4, 4)))

        self.Qv_num  = load_param(f'{config_ns}/forward_speed_weight')
        self.Qv = ca.DM(self.Qv_num)
        
        ## Model Parameters

        self.min_steering = np.radians(load_param(f'{config_ns}/steering_min'))
        self.max_steering = np.radians(load_param(f'{config_ns}/steering_max'))

        self.min_steering_rate = np.radians(load_param(f'{config_ns}/steering_rate_min'))
        self.max_steering_rate = np.radians(load_param(f'{config_ns}/steering_rate_max'))

        self.min_velocity = load_param(f'{config_ns}/velocity_min')
        self.max_velocity = load_param(f'{config_ns}/velocity_max')

        self.min_acceleration = load_param(f'{config_ns}/acceleration_min')
        self.max_acceleration = load_param(f'{config_ns}/acceleration_max')

        # Wheelbase of the vehicle (unit [m]).
        self.L = ca.DM(load_param(f'{config_ns}/wheelbase'))
        
        ## Setup CasADi

        self.opti = ca.Opti()

        self.define_state_and_control_variables()
        self.set_objective_function()
        self.set_state_constraints()
        self.set_control_input_constraints()
        self.set_solver_options()

    def load_param(self, name, value=None):
        self._node.declare_parameter(name, value)
        if value is None:
            assert self._node.has_parameter(name), f'Missing parameter "{name}"'
        return self._node.get_parameter(name).value

    def compute_control(self, state, reference_trajectory):
        """
        Compute the control actions (steering, acceleration) using MPC.
        
        :param state: Current state of the vehicle [x, y, theta, v, delta]
        :param reference_trajectory: Reference trajectory [4, N+1]
        :return: steering, velocity
        """
        # Bound the initial state to respect constraints
        bounded_state = self.bound_initial_state(state)

        # Enlarge the reference trajectory with a fictitious steering value for feasibility.
        # Add one rows of zero to the reference_trajectory matrix.
        reference_trajectory = ca.vertcat(reference_trajectory,ca.DM.zeros(1, reference_trajectory.shape[1]))

        # Set current state and reference trajectory for the active part of the horizon
        self.opti.set_value(self.x_init, bounded_state)
        self.opti.set_value(self.x_ref[:, :self.current_horizon+1], reference_trajectory[:, :self.current_horizon+1])

        # Solve the optimization problem
        self.sol = self.opti.solve()

        # Extract control actions (acceleration and steering rate)
        acceleration = self.sol.value(self.u[0, 0])
        steering_rate = self.sol.value(self.u[1, 0])

        return steering_rate, acceleration
    

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
            return np.array(self.sol.value(self.x))   # Note: to get only optimized one: self.sol.value(self.x[:, :self.current_horizon])
        else: 
            return None

    def define_state_and_control_variables(self):
        # Define state and control variables
        self.x = self.opti.variable(5, self.N + 1)  # state = [x, y, theta, v, steering]
        self.u = self.opti.variable(2, self.N)      # input = [steering_rate, acceleration]

        self.x_init = self.opti.parameter(5)             # Initial state
        self.x_ref = self.opti.parameter(5, self.N + 1)  # Reference trajectory

    def set_objective_function(self):
        # Define the objective function: 
        # J = (x[k]-x_ref[k])^T Q1 (x[k]-x_ref[k]) + (u[k+1] - u[k])^T Q2 (u[k+1] - u[k]) + u[k]^T Q3 u[k] + Qv max(0,-x[3])^2
        self.objective = 0
        for k in range(self.current_horizon):
            # State error term (ignore delta in reference trajectory)
            state_error = self.compute_state_error(self.x[:, k], self.x_ref[:, k])

            # Control input rate of change term (u[k+1] - u[k])
            if k < self.current_horizon - 1:
                input_cost = self.u[:, k+1] - self.u[:, k]
            else:
                input_cost = ca.DM.zeros(self.u.shape[0], 1)

            # Penalize for negative velocity (soft constraint)
            velocity_penalty = ca.fmax(0, -self.x[3, k])  # Penalize if v < 0

            # Accumulate the terms into the objective
            self.objective += (ca.mtimes([state_error.T, self.Q1, state_error]) 
                               + ca.mtimes([input_cost.T, self.Q2, input_cost])
                               + ca.mtimes([self.u[:, k].T, self.Q3, self.u[:, k]])
                               + ca.mtimes([velocity_penalty.T, self.Qv, velocity_penalty]))

        # Final state cost
        final_state_error = self.compute_state_error(self.x[:, self.current_horizon], self.x_ref[:, self.current_horizon])
        self.objective += ca.mtimes([final_state_error.T, self.Qf, final_state_error])

        # Specify type of optimization problem
        self.opti.minimize(self.objective)

    def set_state_constraints(self):
        # Initial state constraint
        self.opti.subject_to(self.x[:, 0] == self.x_init)

        # Vehicle dynamics constraints - Simple kinematic bycicle model
        for k in range(self.N):
            x_next = self.x[0, k] + self.dt * self.x[3, k] * ca.cos(self.x[2, k])                   # x_k+1 = x_k + dt * v_k * cos(theta_k)
            y_next = self.x[1, k] + self.dt * self.x[3, k] * ca.sin(self.x[2, k])                   # y_k+1 = y_k + dt * v_k * sin(theta_k)
            theta_next = self.x[2, k] + self.dt * (self.x[3, k] / self.L) * ca.tan(self.x[4, k])    # theta_k+1 = theta_k + dt * v_k * tan(delta_k) / L
            v_next = self.x[3, k] + self.dt * self.u[0, k]                                          # v_k+1 = v_k + dt * a_k
            delta_next = self.x[4, k] + self.dt * self.u[1, k]                                      # delta_k+1 = delta_k + dt * steering_rate_k

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
        This method checks if the initial state provided to the mpc, which could come from the localization stack,
        is within the allowed bounds. If not, it clamps it to make the optimization problem feasible.
        """
        # Ensure the velocity is within the specified bounds
        clamped_velocity = max(self.min_velocity, min(state[3], self.max_velocity))
        
        # Create a new state with the clamped velocity
        bounded_state = state.copy()
        bounded_state[3] = clamped_velocity
        
        return bounded_state
    
    def compute_state_error(self, x, x_ref):
        """
        Computes the state error between the current state and the reference state.
        Adjusts the yaw error to account for angle wrapping in the range [-π, π].
        """
        state_error = x[0:4] - x_ref[0:4]
        yaw_diff = x[2] - x_ref[2]
        state_error[2] = ca.atan2(ca.sin(yaw_diff), ca.cos(yaw_diff))  # Minimum yaw error 
        return state_error

    def set_new_weight_matrix(self, matrix_name, new_value):
        """
        Dynamically adjust one of the weight matrices.
        Check if the matrix exists as an attribute and if so, update it.
        """
        if hasattr(self, matrix_name):
            try:
                # Overwrite with the new dense matrix
                setattr(self, matrix_name, ca.DM(new_value))
                # Redefine objective function with updated matrix.
                self.set_objective_function()
            except Exception as e:
                print(f"Failed to update {matrix_name}: {e}")
        else:
            print(f"Matrix {matrix_name} does not exist in MPC class.")


    def set_new_prediction_horizon(self, new_horizon):
        """
        Dynamically adjust the active horizon for the optimization problem.
        When the horizon is reduced, you simply freeze unused variables and update the reference trajectory 
        and state for the active part of the horizon. The solver will then only optimize over the active steps.
        """
        self.current_horizon = new_horizon
        
        # Redefine objective function with new horizon.
        self.set_objective_function()


    def reset_parameters(self):
        """
        Reset the core parameters and weight matrices of the MPC instance to their initial values.
        Useful for restoring values to their original state after runtime modifications.
        """
        self.current_horizon = self.N  # Reset to max horizon

        # Reset weight matrices
        self.Q1 = ca.DM(np.array(self.Q1_list).reshape((4, 4)))
        self.Q2 = ca.DM(np.array(self.Q2_list).reshape((2, 2)))
        self.Q3 = ca.DM(np.array(self.Q3_list).reshape((2, 2)))
        self.Qf = ca.DM(np.array(self.Qf_list).reshape((4, 4)))
        self.Qv = ca.DM(self.Qv_num)
        # Reset objective function with initial values.
        self.set_objective_function()
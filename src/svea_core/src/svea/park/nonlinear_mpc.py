#! /usr/bin/env python3

import numpy as np
import casadi as ca
import casadi.tools as ctools
import math

from svea.park.dynamic_model import DynamicModel

'''
Design a nonlinear MPC controller for the SVEA car.

Inputs:
    - Current state: Vector of the form [x, y, yaw, velocity]
    - Trajectory: A list of waypoints that the car should follow  in the form of an array of dimensions (len(traj), number_of_states) [[x1, y1, yaw1], [x2, y2, yaw2], ...]
        - Notice that in the reference to be tracked only the states x, y and yaw are used. A velocity reference is not given.

Outputs: 
    - Control input: Vector of dimensions (number_of_inputs, 1) and has the form of [steering, velocity]
'''
# TODO dynamic references

class NonlinearMPC(object):
    
    def __init__(self, vehicle_name = ''):
        self.vehicle_name = vehicle_name
        self.model = DynamicModel()             # Create a model of the car
        self.dynamics = self.model.dynamics     # Get the dynamics of the car

        # Other variables
        self.traj_x = [0.0]; self.traj_y = [0.0]; self.traj_yaw = [0.0]
        self.target = [0.0, 0.0]    # Needed for the datahandler object
        self.state = None
        self.target_idx = 0
        self.target_radius = 0.25
        self.first_time = True

        self.set_weights()                      # Set the weights for the cost function
        self.set_constraints()                  # Set the constraints for the optimization problem
        self.set_horizon()                      # Set the horizon length
        self.set_solver_options()               # Set the solver options
        self.set_cost_functions()               # Set the cost functions for the optimization problem

        self.generate_ocp()                     # Generate the optimization control problem

        self.warm_start = self.opt_var(0)
        
    def set_weights(self, Q = None, R = None, Qf = None):
        '''
        Inputs:
        - param Q:      Weight matrix for the state error
        - type Q:       np.array((number_of_states, number_of_states))
        - param R:      Weight matrix for the control input
        - type R:       np.array((number_of_inputs, number_of_inputs))
        - param Qf:     Weight matrix for the final state error
        - type Qf:      np.array((number_of_states, number_of_states))
        -
        - Here it is assumed that the number of states is 3 and the number of inputs is 2
        '''
        self.Q = ca.MX(np.diag([2e+2, 2e+2, 6e+0]) if Q is None else Q) # (x, y, yaw)
        self.R = ca.MX(np.diag([8e-1, 3e+0]) if R is None else R)   # (steer, vel)
        self.Qs = ca.MX(np.diag([5e-5, 1e-0]))      # Slack variable for steering, slack variable for velocity
        self.Qf = self.Q
    
    def set_constraints(self, xlb = None, xub = None, ulb = None, uub = None):
        '''
        - param xlb:      Lower bound for the states
        - type xlb:       np.array((number_of_states, 1))
        - param xub:      Upper bound for the states
        - type xub:       np.array((number_of_states, 1))
        - param ulb:      Lower bound for the control inputs
        - type ulb:       np.array((number_of_inputs, 1))
        - param uub:      Upper bound for the control inputs
        - type uub:       np.array((number_of_inputs, 1))
        '''

        self.xlb = np.array([-np.inf, -np.inf, -np.inf]) if xlb is None else ulb        
        self.xub = np.array([np.inf, np.inf, np.inf]) if xub is None else xub
        self.ulb = np.array([self.model.min_steering, -self.model.max_velocity]) if ulb is None else ulb        
        self.uub = np.array([self.model.max_steering, self.model.max_velocity]) if uub is None else uub   
        self.slb = np.array([0e+0, 0e+0]) if ulb is None else ulb
        self.sub = np.array([0e+0, 0e+0]) if uub is None else uub

    def set_horizon(self, N = None):
        '''
        - param N:      Horizon length
        - type N:       int
        '''
        self.N = 10 if N is None else N

    def set_solver_options(self, solver_options = None):
        '''
        - param solver_options:        Options for the solver
        - type solver_options:         dict
        '''
        self.solver_options = {'ipopt.print_level': 0, 'print_time': False, 'verbose': False, 'expand': True } if solver_options is None else solver_options

    def set_cost_functions(self, stage_cost = None, terminal_cost = None):
        '''
        - param stage_cost:     Stage cost function
        - type stage_cost:      CasADi function
        - param terminal_cost:  Terminal cost function
        - type terminal_cost:   CasADi function
        '''

        # Create CasADi variables for the weights
        Q = ca.MX.sym('Q', self.Q.shape[0], self.Q.shape[1])
        R = ca.MX.sym('R', self.R.shape[0], self.R.shape[1])
        Qs = ca.MX.sym('Qs', self.Qs.shape[0], self.Qs.shape[1])
        Qf = ca.MX.sym('Qf', self.Qf.shape[0], self.Qf.shape[1])
        
        # Create CasADi variables for the state vector, input vector and reference vector
        x = ca.MX.sym('x', self.xlb.shape[0], 1)
        u = ca.MX.sym('u', self.ulb.shape[0], 1)
        xr = ca.MX.sym('xr', self.model.n, 1)
        s = ca.MX.sym('s', self.slb.shape[0], 1)    # Slack variables

        ep = x[0:2] - xr[0:2]                           # Position error
        ey = x[2] - xr[2]                               # Yaw error
        ey = ca.if_else(ey > np.pi, ey - 2*np.pi, ey)   # Wrap yaw error between -pi and pi


        # Compute error in slack variable to include in cost function
        e_vec = ca.vertcat(ep, ey)   # Error vector

        # Define the stage cost function
        ln = ca.mtimes( ca.mtimes( (e_vec).T, Q), (e_vec) ) + ca.mtimes( ca.mtimes(u.T, R), u) + ca.mtimes( ca.mtimes(s.T, Qs), s)
        self.stage_cost = ca.Function('ln', [x, xr, Q, u, R, s, Qs], [ln]) if stage_cost is None else stage_cost

        # Define the terminal cost function
        lN = ca.mtimes( ca.mtimes((e_vec).T, Qf), (e_vec) )
        self.terminal_cost = ca.Function('lN', [x, xr, Qf], [lN]) if terminal_cost is None else terminal_cost

    def generate_ocp(self):
        '''
        - Symbollically represent the constraints
        - Symbollically represent the weights
        - 
        - Symbollically represent the accumulated cost over the horizon
        '''

        # Optimization parameters (p)
        x0 = ca.MX.sym('x0', self.model.n)             # CasADi variable for the initial state
        xr = ca.MX.sym('xr', self.model.n*(self.N + 1))  # CasADi variable for the reference
        u0 = ca.MX.sym('u0', self.model.m)               # CasADi variable for the initial control input
        param = ca.vertcat(x0, xr, u0)

        # Optimization variables (x)
        opt_var = ctools.struct_symMX([ctools.entry('u', shape = (self.model.m,), repeat = self.N),
                                       ctools.entry('x', shape = (self.model.n,), repeat = self.N + 1),
                                       ctools.entry('s', shape = (self.model.m,), repeat = self.N)])


        self.opt_var = opt_var                  # TODO: IS THIS REALLY NEEDED?
        self.opt_var_size = opt_var.size

        # Optimization variables boundaries (lbg, ubg)
        self.lbg = opt_var(-np.inf)
        self.ubg = opt_var(np.inf)


        # Set initial values
        obj = ca.MX(0)
        con_eq = []
        con_ineq = []
        con_ineq_lb = []
        con_ineq_ub = []
        con_eq.append(opt_var['x', 0] - x0)


        # Decision variable boundries
        self.optvar_lb = opt_var(-np.inf)
        self.optvar_ub = opt_var(np.inf)

        u_0 = ca.MX.zeros(self.model.m, 1)
        # Loop over the horizon
        for t in range(self.N):
            # Get the current state and input CasADi variables
            x_t = opt_var['x', t]                               # State at time t
            u_t = opt_var['u', t]                               # Current control input at time t
            u_t_prev = opt_var['u', t - 1] if t > 0 else u_0    # Previous control input at time t - 1     
            s_t = opt_var['s', t]                               # Slack variable at time t       
            xr_t = xr[t*self.model.n : (t + 1)*self.model.n]    # Reference at time t

            # Equliaty Constraint: System dynamics
            x_t_next = self.model.dynamics(x_t[0:self.model.n], u_t)    # State at time t + 1

            con_eq.append(x_t_next - opt_var['x', t + 1])

            # Inequality Constraint: Control input bounds
            con_ineq.append(u_t)                                
            con_ineq_lb.append(self.ulb)                        # Lower bounds for the control inputs
            con_ineq_ub.append(self.uub)                        # Upper bounds for the control inputs


            # Inequality Constraint: Control input bounds
            con_ineq.append(u_t - u_t_prev + s_t)                                
            con_ineq_lb.append(self.slb)                        # Lower bounds for the control inputs
            con_ineq_ub.append(self.sub)                        # Upper bounds for the control inputs


            # Inequality Constraint: State bounds
            con_ineq.append(opt_var['x', t])                    
            con_ineq_lb.append(self.xlb)                        # Lower bounds for the states
            con_ineq_ub.append(self.xub)                        # Upper bounds for the states

            # Stage cost
            obj += self.stage_cost(x_t, xr_t, self.Q, u_t, self.R, s_t, self.Qs)


        # Terminal Constraint:              # TODO: Adapt this to our needs
        terminal_constraint = None          # Temporarily used to ignore terminal constraints in the OCP
        if terminal_constraint is not None:
            # Should be a polytope
            H_N = terminal_constraint.A
            if H_N.shape[1] != self.model.n:
                print("Terminal constraint with invalid dimensions.")
                exit()

            H_b = terminal_constraint.b
            con_ineq.append(ca.mtimes(H_N, opt_var['x', self.N]))     # H x <= b
            con_ineq_lb.append(-ca.inf * ca.DM.ones(H_N.shape[0], 1))
            con_ineq_ub.append(H_b)

        # Terminal cost
        obj += self.terminal_cost(opt_var['x', -1], xr_t, self.Qf)


        # Equality constraints
        num_eq_con = ca.vertcat(*con_eq).size1()
        con_eq_lb = np.zeros((num_eq_con,))
        con_eq_ub = np.zeros((num_eq_con,))

        # Set constraints
        con = ca.vertcat(*(con_eq + con_ineq))

        self.con_lb = ca.vertcat(con_eq_lb, *con_ineq_lb)
        self.con_ub = ca.vertcat(con_eq_ub, *con_ineq_ub)

        # Create the NLP solver
        nlp = dict(x = opt_var, f = obj, g = con, p = param)
        self.solver = ca.nlpsol('mpc_solver', 'ipopt', nlp, self.solver_options)

    def solve_mpc(self, x0, ref, u0 = None):
        '''
        Solves a direct multiple-shooting optimal control problem
        '''

        u0 = np.zeros(self.model.m)

        # Repeat the reference point for each time step
        xr_ = np.tile(ref, (self.N + 1, 1))
        xr = xr_.reshape( (self.N + 1)*self.model.n, 1)

        # Add zeros for the integral states to the initial state x0
        x0_ = np.zeros((self.model.n,))
        x0_[0:self.model.n] = x0

        self.optvar_x0 = np.full((1, self.model.n), x0_.T)              # Set the initial state
     
        # Initial guess of the warm start variables
        # self.optvar_init = self.opt_var(0)
        self.optvar_init = self.warm_start
        self.optvar_init['x', 0] = self.optvar_x0[0]        

        param = ca.vertcat(x0_, xr, u0)                      # Set the parameters

        args = dict(x0=self.optvar_init,
                    lbx=self.optvar_lb,
                    ubx=self.optvar_ub,
                    lbg=self.con_lb,
                    ubg=self.con_ub,
                    p=param)

        # Solve NLP
        sol = self.solver(**args)
        optvar = self.opt_var(sol['x'])

        self.warm_start = optvar

        return optvar['u']   

    def compute_control(self, state: object):
        self.state = state                              # Globally update the state of the system
        x0 = np.asarray([state.x, state.y, state.yaw])  # Extract initial state

        # update reference point only when reached
        if self.target_reached or self.first_time:
            self.xr = self.update_reference(x0)
            self.first_time = False


        # SOLVE REF ISSUES FOR ANGLE:
        a = self.xr[2] - x0[2]
        if a > np.pi:
            a -= 2*np.pi
        elif a < -np.pi:
            a += 2*np.pi
        self.xr[2] = x0[2] + a


        # Solve the optimization problem and retrieve the optimal control input sequence
        u = self.solve_mpc(x0, self.xr)
        u = np.array(u)

        # Extract the first control input pair from the sequence
        u0 = u[0, :]

        # self.print_status(self.xr, np.array([self.state.x, self.state.y, self.state.yaw]), u0)

        return u0[0], u0[1]   # steering[0, velocity[1]



    @property
    def target_reached(self):
        dx = self.target[0] - self.state.x
        dy = self.target[1] - self.state.y
        d = math.sqrt(dx ** 2 + dy ** 2)
        return d <= self.target_radius



    def print_status(self, goal, state, u):
        # Calculate error and put in np array
        error = np.array([goal[0] - state[0], goal[1] - state[1], goal[2] - state[2]])

        print(f'-----------------------------------------------------------------------------------------------')
        print(f'|          State         |           Goal           |         Error           |      Input    |')
        print(f'-----------------------------------------------------------------------------------------------')
        print(f'|   x   |   y   |  yaw  | x_goal | y_goal |yaw_goal|error_x|error_y|error_yaw| steer |  vel  |')
        print(f'| {state[0]:+.2f} | {state[1]:+.2f} | {state[2]:+.2f} | {goal[0]:+.2f}  | {goal[1]:+.2f}  | {goal[2]:+.2f}  | {error[0]:+.2f} | {error[1]:+.2f} |  {error[2]:+.2f} | {u[0][0]:+.2f} | {u[1][0]:+.2f} |')
        print(f'-----------------------------------------------------------------------------------------------')


    def update_reference(self, state):
        '''
        - param state: Current state of the vehicle
        - type state: np.array([x, y, yaw])
        '''

        # Find next target point
        xr = self.find_target()

        # self.xr = np.array([10.3, 11.7, 0.9])       # TODO: Temporary
        
        return xr


    def find_target(self): # Find the next target point
        new_idx = self._calc_target_index()
        self.target_idx = min(max(self.target_idx, new_idx), len(self.traj_x) - 2)
        tx = self.traj_x[self.target_idx]
        ty = self.traj_y[self.target_idx]
        tyaw = self.traj_yaw[self.target_idx]

        self.target = np.array([tx, ty])    # For the datahandler        
        return np.array([tx, ty, tyaw])

        # return (tx, ty, tyaw)


    def _calc_target_index(self):
        k = 0.2#0.1 #0.2# 0.6*0.1  # look forward gain
        Lfc = 0.15 #0.2 #0.4 # 0.2*0.075  # look-ahead distance
                
        # Find index of nearest point 
        dx = [self.state.x - icx for icx in self.traj_x]
        dy = [self.state.y - icy for icy in self.traj_y]
        distances = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        
        try:
            dist = min(distances) # Distance to closest point
            idx = distances.index(dist)
        except:
            dist = 0
            print(f'NLMPC - ERROR: DISTANCES LIST IS EMPTY')
        #idx = distances.index(dist)
        Lf = k * self.state.v + Lfc

        # Make sure the distance to the closest point greater than Lf
        while dist < Lf and (idx + 1) < len(self.traj_x): 
            dx = self.traj_x[idx + 1] - self.traj_x[idx]
            dy = self.traj_y[idx + 1] - self.traj_y[idx]
            dist += math.sqrt(dx ** 2 + dy ** 2)
            idx += 1

        return idx


    def initialize_state(self, state):
        self.state = state

        
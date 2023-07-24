#! /usr/bin/env python3

'''
This class abides by the structure imposed by the SimSVEA class.
In particular:
    - Has a state variable 'self.state' of type 'VehicleState'
    - Has a method 'self.update()' that updates 'self.state' and takes as input:
        - A steering input  [rad]
        - A velocity input  [m/s]
        - A time step       [s]
'''

import rospy
import casadi as ca
import numpy as np
import math

from svea.states import VehicleState as SVEAVehicleState

class DynamicModel(object):
    def __init__(self,
                 state: SVEAVehicleState = None):
        '''
        - param state: The initial state of the vehicle
        - type state: From svea.states Class VehicleState
        '''
        # SVEA Car parameters
        self.L = 0.324                          # Wheel base [m]
        self.max_steering = 40 * math.pi / 180  # Max steering angle [rad]
        self.min_steering = -40 * math.pi / 180 # Min steering angle [rad]
        self.max_velocity = 0.8#0.5                 # Max velocity [m/s]
 
        # Model parameters
        self.dt = 0.01                          # Time step [s]
        self.n = 3                              # Number of states [x, y, theta]
        self.m = 2                              # Number of inputs [steering, velocity]

        # Initialize state
        self.state = state if state is not None else SVEAVehicleState() # Initialize Vehicle state
        self.state.time_stamp = rospy.get_rostime()

        # Initialize input
        self.u = np.zeros((self.m, 1))          # Input vector

        self.dynamics = self.rk4_integrator(self.nonlinear_dynamics)


    def nonlinear_dynamics(self, x, u):
        """
        Bicycle nonlinear dynamics.

        :param x: state
        :type x: ca.MX
        :param u: control input
        :type u: ca.MX
        :return: state time derivative
        :rtype: ca.MX
        """

        # Extract states
        px = x[0]
        py = x[1]
        q = x[2]

        # Extract control inputs
        steer = u[0]    # Steering angle
        vel = u[1]      # Velocity

        # Model
        pxdot = ca.MX.zeros(1, 1)
        pydot = ca.MX.zeros(1, 1)
        qdot = ca.MX.zeros(1, 1)
        

        pxdot = vel @ ca.cos(q)
        pydot = vel @ ca.sin(q)
        qdot = vel @ ca.tan(steer) / self.L

        dxdt = [pxdot, pydot, qdot]

        return ca.vertcat(*dxdt)


    def rk4_integrator(self, dynamics):
        """
        Runge-Kutta 4th Order discretization.
        :param x: state
        :type x: ca.MX
        :param u: control input
        :type u: ca.MX
        :return: state at next step
        :rtype: ca.MX
        """
        x0 = ca.MX.sym('x0', self.n, 1)
        u = ca.MX.sym('u', self.m, 1)

        x = x0

        k1 = dynamics(x, u)
        k2 = dynamics(x + self.dt / 2 * k1, u)
        k3 = dynamics(x + self.dt / 2 * k2, u)
        k4 = dynamics(x + self.dt * k3, u)
        xdot = x0 + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        fun_options = {
            "jit": False,
            "jit_options": {"flags": ["-O2"]}
        }        

        rk4 = ca.Function('RK4', [x0, u], [xdot], fun_options)

        return rk4
    
    # Interface with the SimSvea class
    def update(self, targ_steering, targ_velocity, dt):
        '''
        - param targ_steering:  The steering input [rad]
        - type targ_steering:   float
        - param targ_velocity:  Target velocity [m/s]
        - type targ_velocity:   float
        - param dt:             The time step [s]
        - type dt:              float [s]
        '''

        # Extract state variables and input variables
        x = self.state.x; y = self.state.y; yaw = self.state.yaw; v = self.state.v
        z = np.array([x, y, yaw])
        u = np.array([targ_steering, v])

        # Predict accelartion
        accel = self._sim_esc(self.state.v, targ_velocity)
        # Update state using nonlinear model
        self.nonlinear_update(self.state, accel, targ_steering, dt)

        self.state.time_stamp += rospy.Duration.from_sec(dt)

    def nonlinear_update(self, state, accel, steering, dt):
        # Update state using simple bicycle model dynamics
        steering = np.clip(steering, self.min_steering, self.max_steering)
        # Extract state variables from VehicleState
        yaw = self.state.yaw; v = self.state.v
        # Update state
        state.x += v * np.cos(yaw) * dt
        state.y += v * np.sin(yaw) * dt
        state.yaw += v / self.L * np.tan(steering) * dt
        state.v += accel * dt


    # Auxiliary functions
    def _sim_esc(self, velocity, target_velocity):
        '''
        This function has been copied from Frank Jiang's implementation of the SimpleBicycle class.
        '''
        # Simulate ESC dynamics
        tau = 0.1   # Time constant [s]
        return 1/tau * (target_velocity - velocity) 
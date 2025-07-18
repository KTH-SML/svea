#! /usr/bin/env python3

import numpy as np
import math
import tf_transformations as tf
from svea_core.models.bicycle import Bicycle4DWithESC
from svea_core.interfaces import LocalizationInterface, ActuationInterface
try:
    from svea_mocap.mocap import MotionCaptureInterface
except ImportError:
    pass
from svea_core.controllers.mpc import MPC_casadi
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray, PoseStamped

from svea_core import rosonic as rx

class mpc(rx.Node):
    is_sim = rx.Parameter(True)
    state = rx.Parameter([-3.0, 0.0, 0.0, 0.0])  # x, y, yaw, velocity
    mpc_freq = rx.Parameter(10)  # Hz
    delta_s = rx.Parameter(5)  # m
    mpc_config_ns = rx.Parameter('/mpc')
    target_speed = rx.Parameter(0.5)  # m/s
    prediction_horizon = rx.Parameter(10)
    final_state_weight_matrix = rx.Parameter(None)  # Weight matrix for the final state in MPC


    ## MPC parameters 
    GOAL_REACHED_DIST = 0.2   # The distance threshold (in meters) within which the goal is considered reached.
    GOAL_REACHED_YAW = 0.2    # The yaw angle threshold (in radians) within which the goal orientation is considered reached.
    UPDATE_MPC_PARAM = True   # A flag indicating if the MPC parameters can be updated when the system is approaching the target.
    RESET_MPC_PARAM = False   # A flag indicating if the MPC parameters should be reset when the system is moving away from the target.
    predicted_state = None
    current_horizon = prediction_horizon

    ## Static Planner parameters
    APPROACH_TARGET_THR = 5   # The distance threshold (in meters) to define when the system is "approaching" the target.
    NEW_REFERENCE_THR = 1     # The distance threshold (in meters) to update the next intermediate reference point. 
    goal_pose = None
    static_path_plan = np.empty((3, 0))
    current_index_static_plan = 0
    is_last_point = False

    ## Other parameters
    steering = 0
    velocity = 0
    state = []

    dt =0.01

    def on_startup(self):

        ## Define the unitless steering biases for each SVEA.
        ## These values represent the measured steering actuations when the SVEA is not actually steering.
        self.unitless_steering_map = {
            "svea0": 28,
            "svea7": 7
        }

        self.mpc = MPC_casadi
        self.get_logger().info(f"Using MPC frequency: {self.final_state_weight_matrix} Hz")
        self.DELTA_TIME = 1.0/self.mpc_freq
        print(self.DELTA_TIME)

        self.create_timer(self.DELTA_TIME, self.loop)

        # if not self.is_sim:
        #     svea_name = self.SVEA_MOCAP_NAME.lower()  # Ensure case-insensitivity  
        #     unitless_steering = self.unitless_steering_map.get(svea_name, 0)  # Default to 0 if not found
        #     PERC_TO_LLI_COEFF = 1.27
        #     MAX_STEERING_ANGLE = 40 * math.pi / 180
        #     steer_percent = unitless_steering / PERC_TO_LLI_COEFF
        #     self.steering_bias = (steer_percent / 100.0) * MAX_STEERING_ANGLE
        # else:
        #     self.steering_bias = 0

    def loop(self):
        self.get_logger().info(f"Using MPC frequency: {self.final_state_weight_matrix} Hz")

    def load_param(self, name, value=None):
        self.declare_parameter(name, value)
        if value is None:
            assert self.has_parameter(name), f'Missing parameter "{name}"'
        return self.get_parameter(name).value
        
    def create_simulator_and_SVEAmanager(self):
        # Create simulators, models, managers, etc.
        if self.IS_SIM:

            # simulator need a model to simulate
            self.sim_model = Bicycle4DWithESC()

        # start the SVEA manager (needed for both sim and real world)
        if not self.IS_SIM:
            self.svea.localizer.update_name(self.SVEA_MOCAP_NAME)

        self.svea.start(wait=True)

        # everything ready to go -> unpause simulator
        if self.IS_SIM:
            self.simulator.toggle_pause_simulation()

if __name__ == '__main__':
    mpc.main()
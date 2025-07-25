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
from svea_core.controllers.mpc import MPC

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.clock import Clock
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray

from svea_core import rosonic as rx

qos_subber = QoSProfile(depth=10 # Size of the queue 
                        )

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

class mpc_path_tracking(rx.Node):
    is_sim = rx.Parameter(True)
    state = rx.Parameter([-3.0, 0.0, 0.0, 0.0])  # x, y, yaw, velocity
    mpc_freq = rx.Parameter(10)  # Hz
    svea_mocap_name = rx.Parameter('svea7')
    target_speed = rx.Parameter(0.5)  # m/s
    circle_radius = rx.Parameter(5.0)  # m
    circle_center_x = rx.Parameter(0.0)  # m
    circle_center_y = rx.Parameter(0.0)  # m
    mpc_config_ns = rx.Parameter('/mpc')
    time_step = rx.Parameter(0.2)  # m
    prediction_horizon = rx.Parameter(5)

<<<<<<< HEAD
    actuation = ActuationInterface()
    localizer = LocalizationInterface()

    ##MPC parameters
    predicted_state = None
    mpc_last_time = 0
    WRAP_AROUND_ENABLED = True
    current_index_static_plan = 0

    steering = 0
    velocity = 0
    state = []

    steering_pub = rx.Publisher(Float32, '/target_steering_angle', qos_profile=qos_subber)
    velocity_pub = rx.Publisher(Float32, '/target_speed', qos_profile=qos_subber)
    velocity_measured_pub = rx.Publisher(Float32, '/measured_speed', qos_profile=qos_subber)
    predicted_trajectory_pub = rx.Publisher(PoseArray, '/predicted_path', qos_profile=qos_pubber)
    static_trajectory_pub = rx.Publisher(PoseArray, '/static_path', qos_profile=qos_pubber)

    def on_startup(self):
        """
        Initialize the MPC controller and set up the static path plan.
        """

        self.controller = MPC(self)
        self.mpc_dt = 1.0 / self.mpc_freq
        self.initial_horizon = self.prediction_horizon
        self.current_horizon = self.prediction_horizon
        self.static_path_plan = np.empty((3, 0))
        self.mpc_propagation_dt = self.time_step

        ## Define the unitless steering biases for each SVEA.
        ## These values represent the measured steering actuations when the SVEA is not actually steering.
        self.unitless_steering_map = {
            "svea0": 28,
            "svea7": 7
        }
        if self.is_sim is False:
            svea_name = self.SVEA_MOCAP_NAME.lower()  # Ensure case-insensitivity  
            unitless_steering = self.unitless_steering_map.get(svea_name, 0)  # Default to 0 if not found
            PERC_TO_LLI_COEFF = 1.27
            MAX_STEERING_ANGLE = 40 * math.pi / 180
            steer_percent = unitless_steering / PERC_TO_LLI_COEFF
            self.steering_bias = (steer_percent / 100.0) * MAX_STEERING_ANGLE
        else:
            self.steering_bias = 0

        self.create_timer(self.mpc_dt, self.loop)

    def loop(self):
        if self.static_path_plan.size == 0:
            self.generate_static_circle_path()
            self.mpc_last_time = Clock().now().nanoseconds / 1e9  # Initialize the last time to current time
        else:
            None
        # Retrieve current state from SVEA localization
        self.state = self.localizer.get_state()
        # Run the MPC if the static path plan is available
        if self.static_path_plan.size > 0:
            current_time = Clock().now().nanoseconds / 1e9 # Get current time in seconds
            measured_dt = current_time - self.mpc_last_time
            if measured_dt >= self.mpc_dt:
                reference_trajectory = self.get_mpc_current_reference()
                # Run the MPC to compute control
                steering_rate, acceleration = self.controller.compute_control(
                    [self.state[0], self.state[1], self.state[2], self.state[3], self.steering], reference_trajectory
                )
                self.steering += steering_rate * measured_dt
                self.velocity += acceleration * measured_dt
                self.predicted_state = self.controller.get_optimal_states()
                self.mpc_last_time = current_time

        # Visualization data and send control
        self.actuation.send_control(self.steering + self.steering_bias, self.velocity)

    
    def get_mpc_current_reference(self):
        """
        Retrieves the current reference state for the MPC based on the SVEA's current position.

        This function finds the closest point on the static path to the current vehicle state.
        It then generates the reference trajectory (`x_ref`) consisting of points from the static path,
        starting from the point after the closest one up to `initial_horizon + 1` steps ahead.

        If the vehicle is close to the end of the static path, the function ensures that `x_ref`
        has enough points by either wrapping around the static path (if `WRAP_AROUND_ENABLED` is True, useful for infinite paths)
        or repeating the last point until `x_ref` contains `initial_horizon + 1` points.

        The function also appends an additional row to `x_ref` with all elements equal to `self.target_speed`, representing
        the desired speed for each reference point.

        Returns:
            x_ref (numpy.ndarray): The reference trajectory for the MPC, with the desired speed appended as the last row.
        """
        self.current_index_static_plan = self.find_closest_point_index()
        start_index = self.current_index_static_plan + 1 
        end_index = start_index + self.initial_horizon + 1
        if end_index > self.N:
            if self.WRAP_AROUND_ENABLED:
                # Wrap around by splitting the reference points into two segments
                remaining_points = self.static_path_plan[:, start_index:self.N]
                wrapped_points = self.static_path_plan[:, :end_index - self.N]
                x_ref = np.concatenate((remaining_points, wrapped_points), axis=1)
            else:
                x_ref = self.static_path_plan[:, start_index:self.N]
                # Ensure x_ref has the same number of columns as initial_horizon + 1
                while x_ref.shape[1] < self.initial_horizon + 1:
                    x_ref = np.concatenate((x_ref, self.static_path_plan[:, -1:]), axis=1)
        else:
            x_ref = self.static_path_plan[:, start_index:end_index]

        # Append a row with the desired speed for each point
        target_speed_row = np.full((1, x_ref.shape[1]), self.target_speed)
        x_ref = np.concatenate((x_ref, target_speed_row), axis=0)

        return x_ref
    

    def find_closest_point_index(self):
        """
        Finds the index of the closest point in the static path to the current state.
        """
        distances = np.linalg.norm(self.static_path_plan[:2, :] - np.array(self.state[:2])[:, None], axis=0)
        return np.argmin(distances)
    

    def generate_static_circle_path(self):
        """
        Generates a static circular path with a specified number of equally spaced points.
        
        The first point is positioned at theta = -π and the path proceeds counterclockwise.
        The discretization of the path is carefully chosen to ensure compatibility with 
        the MPC parameters, specifically the discretization time and the desired traversal speed.
        
        To ensure meaningful dynamic propagation within the MPC, the spatial step size of 
        the path (ds_path) should satisfy the condition:
        
            |ds_path - ds_mpc| < epsilon

        where `ds_path` is the spatial step size of the path, `ds_mpc = v_des * discretization_time` 
        is the MPC's spatial step size, and `epsilon` is a small tolerance value.   
        """
        path_length = 2 * np.pi * self.circle_radius
        ds_des = self.target_speed * self.mpc_propagation_dt
        self.N = int((path_length / ds_des) * 1.15)  # Add 15% buffer
        theta_values = np.linspace(-math.pi ,  math.pi , self.N, endpoint=False)
        x_values = self.circle_center_x + self.circle_radius * np.cos(theta_values)
        y_values = self.circle_center_y + self.circle_radius * np.sin(theta_values)
        yaw_values = np.arctan2(np.diff(y_values, append=y_values[0]), np.diff(x_values, append=x_values[0]))
        self.static_path_plan = np.vstack((x_values, y_values, yaw_values))

    def generate_static_line_path(self):
        """
        Generates a straight-line path with equally spaced points.
        The line is horizontal (y = const) with x values ranging between specified limits.
        """
        x_start = self.circle_center_x + self.circle_radius
        x_end = self.circle_center_x - self.circle_radius
        y_const = self.circle_center_y

        x_values = np.linspace(x_start, x_end, self.N)
        y_values = np.full_like(x_values, y_const)
        yaw_values = np.full_like(x_values, math.pi)
        self.static_path_plan = np.vstack((x_values, y_values, yaw_values))



if __name__ == '__main__':
    mpc_path_tracking.main()
=======
    
>>>>>>> bc2b7b5 (mpc goal position complete)

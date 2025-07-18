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
from rclpy.qos import QoSProfile

from svea_core import rosonic as rx

qos_subber = QoSProfile(depth=10 # Size of the queue 
                        )

class mpc(rx.Node):
    is_sim = rx.Parameter(True)
    state = rx.Parameter([-3.0, 0.0, 0.0, 0.0])  # x, y, yaw, velocity
    mpc_freq = rx.Parameter(10)  # Hz
    delta_s = rx.Parameter(5)  # m
    mpc_config_ns = rx.Parameter('/mpc')
    target_speed = rx.Parameter(0.5)  # m/s
    prediction_horizon = rx.Parameter(5)
    final_state_weight_matrix = rx.Parameter(None)  # Weight matrix for the final state in MPC

    actuation = ActuationInterface()
    localizer = LocalizationInterface()

    ## MPC parameters 
    GOAL_REACHED_DIST = 0.2   # The distance threshold (in meters) within which the goal is considered reached.
    GOAL_REACHED_YAW = 0.2    # The yaw angle threshold (in radians) within which the goal orientation is considered reached.
    UPDATE_MPC_PARAM = True   # A flag indicating if the MPC parameters can be updated when the system is approaching the target.
    RESET_MPC_PARAM = False   # A flag indicating if the MPC parameters should be reset when the system is moving away from the target.
    predicted_state = None

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

    @rx.Subscriber(PoseStamped, 'mpc_target', qos_subber)
    def mpc_target_callback(self, msg):
        """
        Callback function that sets a new goal position and calculates a trajectory.
        :param msg: PoseStamped message containing the goal position.
        """
        # Set the goal position and log the new goal
        self.goal_pose = msg
        self.get_logger().info(f"New goal position received: ({self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y})")
        # Compute trajectory (straight line from current position to goal)
        self.compute_trajectory()
    

    def on_startup(self):

        ## Define the unitless steering biases for each SVEA.
        ## These values represent the measured steering actuations when the SVEA is not actually steering.
        self.unitless_steering_map = {
            "svea0": 28,
            "svea7": 7
        }

        self.controller = MPC_casadi(self)
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
        
    def create_simulator_and_SVEAmanager(self):
        # Create simulators, models, managers, etc.
        if self.is_sim:

            # simulator need a model to simulate
            self.sim_model = Bicycle4DWithESC()

        # everything ready to go -> unpause simulator
        if self.is_sim:
            self.simulator.toggle_pause_simulation()

    def compute_trajectory(self):
        """
        Compute a straight-line trajectory from the current position to the goal using DELTA_S,
        including the heading for each point, and publish the path.
        """
        if not self.goal_pose or not self.state:
            self.get_logger().warning("Missing goal or current state for trajectory computation.")
            return
        # Reset previous trajectory related variables
        self.current_index_static_plan = 0
        self.is_last_point = False
        self.static_path_plan = np.empty((3, 0))
        # reset control actions
        self.velocity = 0
        self.steering = 0
        # reset mpc parameters
        self.UPDATE_MPC_PARAM = True  
        self.RESET_MPC_PARAM = False
        self.mpc_last_time = self.clock.Clock().now().to_msg()
        self.controller.reset_parameters()

        # Calculate the straight-line trajectory between current state and goal position
        start_x, start_y = self.state[0], self.state[1]
        goal_x, goal_y = self.goal_pose.pose.position.x, self.goal_pose.pose.position.y
        distance = self.compute_distance([start_x, start_y], [goal_x, goal_y])
        goal_yaw = self.get_yaw_from_pose(self.goal_pose)

        # Compute intermediate points at intervals of DELTA_S
        num_points = int(distance // self.delta_s)

        for i in range(num_points):
            ratio = ((i+1) * self.DELTA_S) / distance
            x = start_x + ratio * (goal_x - start_x)
            y = start_y + ratio * (goal_y - start_y)
            
            # Calculate the heading for this point
            heading = math.atan2(goal_y - start_y, goal_x - start_x)
            
            # Stack the computed point as a new column in the array
            new_point = np.array([[x], [y], [heading]])
            self.static_path_plan = np.hstack((self.static_path_plan, new_point))

        # Calculate distance between the last appended point and the goal point.
        if self.static_path_plan.size != 0:
            last_appended_x = self.static_path_plan[0, -1]
            last_appended_y = self.static_path_plan[1, -1]    
            distance = self.compute_distance([last_appended_x, last_appended_y], [goal_x, goal_y])

            # If the distance to the goal is too small, replace the last point with the goal directly.
            if distance < self.DELTA_S / 2:
                # Replace the last appended point with the goal point
                self.static_path_plan[:, -1] = np.array([goal_x, goal_y, goal_yaw])
            else:
                # Otherwise, append the last point as usual
                new_point = np.array([[goal_x], [goal_y], [goal_yaw]])
                self.static_path_plan = np.hstack((self.static_path_plan, new_point))
        else:
            # Otherwise, append the last point as usual
            new_point = np.array([[goal_x], [goal_y], [goal_yaw]])
            self.static_path_plan = np.hstack((self.static_path_plan, new_point))
        
    def compute_distance(self,point1,point2):
        return np.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)
    
    def get_yaw_from_pose(self,pose_stamped):
        """
        Extracts the yaw from a PoseStamped message.
        """
        # Convert the quaternion to Euler angles
        orientation = pose_stamped.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        
        # Use tf to convert quaternion to Euler angles
        euler = tf.euler_from_quaternion(quaternion)
        
        # Return the yaw
        return euler[2]  

if __name__ == '__main__':
    mpc.main()
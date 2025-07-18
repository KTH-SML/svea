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
<<<<<<< HEAD
from svea_core.controllers.mpc import MPC
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.clock import Clock

from svea_core import rosonic as rx

qos_subber = QoSProfile(depth=10 # Size of the queue 
                        )

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

=======
from svea_core.controllers.mpc import MPC_casadi
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray, PoseStamped

from svea_core import rosonic as rx

>>>>>>> 8b92c94 (added mpc control and example, but still in working progress)
class mpc(rx.Node):
    is_sim = rx.Parameter(True)
    state = rx.Parameter([-3.0, 0.0, 0.0, 0.0])  # x, y, yaw, velocity
    mpc_freq = rx.Parameter(10)  # Hz
    delta_s = rx.Parameter(5)  # m
    mpc_config_ns = rx.Parameter('/mpc')
    target_speed = rx.Parameter(0.5)  # m/s
<<<<<<< HEAD
    prediction_horizon = rx.Parameter(5)
    final_state_weight_matrix = rx.Parameter(None)  # Weight matrix for the final state in MPC

    actuation = ActuationInterface()
    localizer = LocalizationInterface()
=======
    prediction_horizon = rx.Parameter(10)
    final_state_weight_matrix = rx.Parameter(None)  # Weight matrix for the final state in MPC

>>>>>>> 8b92c94 (added mpc control and example, but still in working progress)

    ## MPC parameters 
    GOAL_REACHED_DIST = 0.2   # The distance threshold (in meters) within which the goal is considered reached.
    GOAL_REACHED_YAW = 0.2    # The yaw angle threshold (in radians) within which the goal orientation is considered reached.
    UPDATE_MPC_PARAM = True   # A flag indicating if the MPC parameters can be updated when the system is approaching the target.
    RESET_MPC_PARAM = False   # A flag indicating if the MPC parameters should be reset when the system is moving away from the target.
    predicted_state = None
<<<<<<< HEAD
=======
    current_horizon = prediction_horizon
>>>>>>> 8b92c94 (added mpc control and example, but still in working progress)

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

<<<<<<< HEAD
    steering_pub = rx.Publisher(Float32, '/target_steering_angle', qos_profile=qos_subber)
    velocity_pub = rx.Publisher(Float32, '/target_speed', qos_profile=qos_subber)
    velocity_measured_pub = rx.Publisher(Float32, '/measured_speed', qos_profile=qos_subber)
    predicted_trajectory_pub = rx.Publisher(PoseArray, '/predicted_path', qos_profile=qos_pubber)
    static_trajectory_pub = rx.Publisher(PoseArray, '/static_path', qos_profile=qos_pubber)

    @rx.Subscriber(PoseStamped, '/mpc_target', qos_pubber)
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
    

=======
>>>>>>> 8b92c94 (added mpc control and example, but still in working progress)
    def on_startup(self):

        ## Define the unitless steering biases for each SVEA.
        ## These values represent the measured steering actuations when the SVEA is not actually steering.
        self.unitless_steering_map = {
            "svea0": 28,
            "svea7": 7
        }

<<<<<<< HEAD
        self.controller = MPC(self)
        self.DELTA_TIME = 1.0/self.mpc_freq
        self.initial_horizon = self.prediction_horizon
        self.initial_Qf = self.final_state_weight_matrix

        self.create_timer(self.DELTA_TIME, self.loop)

        if not self.is_sim:
            svea_name = self.SVEA_MOCAP_NAME.lower()  # Ensure case-insensitivity  
            unitless_steering = self.unitless_steering_map.get(svea_name, 0)  # Default to 0 if not found
            PERC_TO_LLI_COEFF = 1.27
            MAX_STEERING_ANGLE = 40 * math.pi / 180
            steer_percent = unitless_steering / PERC_TO_LLI_COEFF
            self.steering_bias = (steer_percent / 100.0) * MAX_STEERING_ANGLE
        else:
            self.steering_bias = 0

    def loop(self):
        # Retrieve current state from SVEA localization
        self.state = self.localizer.get_state()
        # If a static path plan has been computed, run the mpc.
        if self.static_path_plan.size > 0 :
            # If enough time has passed, run the MPC computation
            current_time = Clock().now().to_msg()
            time_diff_sec = (current_time.sec - self.mpc_last_time.sec) + \
                (current_time.nanosec - self.mpc_last_time.nanosec) / 1e9
            measured_dt = time_diff_sec
                        # current_time - self.mpc_last_time
            if measured_dt >= self.DELTA_TIME:
                reference_trajectory, distance_to_next_point = self.get_mpc_current_reference()
                if self.is_last_point and distance_to_next_point <= self.APPROACH_TARGET_THR and self.UPDATE_MPC_PARAM:
                    # Update the prediction horizon and final state weight matrix only once when approaching target to achieve better parking.
                    new_Qf = np.array([70, 0, 0, 0,
                                        0, 70, 0, 0,
                                        0, 0, 20, 0,
                                        0, 0, 0, 0]).reshape((4, 4))
                    self.controller.set_new_weight_matrix('Qf', new_Qf)
                    self.UPDATE_MPC_PARAM = False
                    self.RESET_MPC_PARAM = True  # Allow resetting when moving away

                elif self.is_last_point and distance_to_next_point > self.APPROACH_TARGET_THR and self.RESET_MPC_PARAM:
                    # Reset to initial values only once when moving away from target
                    self.current_horizon = self.initial_horizon
                    self.controller.set_new_prediction_horizon(self.initial_horizon)
                    self.controller.set_new_weight_matrix('Qf', self.initial_Qf)
                    self.UPDATE_MPC_PARAM = True  # Allow updating again when re-approaching
                    self.RESET_MPC_PARAM = False  # Prevent repeated resetting

                if  not self.is_goal_reached(distance_to_next_point):
                    # Run the MPC to compute control
                    steering_rate, acceleration = self.controller.compute_control([self.state[0],self.state[1],self.state[2],self.velocity,self.steering], reference_trajectory)
                    self.steering += steering_rate * measured_dt
                    self.velocity += acceleration * measured_dt  
                    self.predicted_state = self.controller.get_optimal_states()
                else:
                    # Stop the vehicle if the goal is reached
                    self.steering, self.velocity = 0, 0
                # Update the last time the MPC was computed
                self.mpc_last_time = current_time
            
        # Publish the latest control target and the estimated speed( from mocap or indoors loc. or outdoors loc.).
        # self.publish_to_foxglove(self.steering, self.velocity, self.state[3])
        # Visualization data and send control
        self.actuation.send_control(self.steering + self.steering_bias, self.velocity) 
        # self.svea.visualize_data()
        

    def publish_to_foxglove(self,target_steering,target_speed,measured_speed):
        self.steering_pub.publish(target_steering)
        self.velocity_pub.publish(target_speed)
        self.velocity_measured_pub.publish(measured_speed)

    def get_mpc_current_reference(self):
        """
        Retrieves the current reference state for the MPC based on the SVEA's current position.

        Updates the current index in the static path plan if the robot is close to the next point. If at the last point, 
        it calculates the distance to that point instead.

        Returns:
            tuple: (x_ref, distance_to_next_point), where x_ref is the reference state for the prediction horizon
            (shape: [4, N+1]) and distance_to_next_point is the distance to the next or last reference point.
        """
        if self.is_last_point is False:
            distance_to_next_point = self.compute_distance(self.state,self.static_path_plan[:,self.current_index_static_plan])
            if self.current_index_static_plan == self.static_path_plan.shape[1] - 1:
                self.is_last_point = True
            if distance_to_next_point < self.NEW_REFERENCE_THR and not self.is_last_point:
                self.current_index_static_plan += 1  
            reference_state = self.static_path_plan[:,self.current_index_static_plan]
            x_ref = np.tile(reference_state, (self.initial_horizon+1, 1)).T 
            target_speed_row = np.full((1, x_ref.shape[1]), self.target_speed)                
            x_ref = np.concatenate((x_ref, target_speed_row), axis=0)
            return x_ref, distance_to_next_point

        else:
            distance_to_last_point = self.compute_distance(self.state,self.static_path_plan[:,-1])
            reference_state = self.static_path_plan[:,-1]
            x_ref = np.tile(reference_state, (self.initial_horizon+1, 1)).T
            target_speed_row = np.full((1, x_ref.shape[1]), self.target_speed)
            x_ref = np.concatenate((x_ref, target_speed_row), axis=0)
            return x_ref, distance_to_last_point

    def compute_trajectory(self):
        """
        Compute a straight-line trajectory from the current position to the goal using delta_s,
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
        self.mpc_last_time = Clock().now().to_msg()
        self.controller.reset_parameters()

        # Calculate the straight-line trajectory between current state and goal position
        start_x, start_y = self.state[0], self.state[1]
        goal_x, goal_y = self.goal_pose.pose.position.x, self.goal_pose.pose.position.y
        distance = self.compute_distance([start_x, start_y], [goal_x, goal_y])
        goal_yaw = self.get_yaw_from_pose(self.goal_pose)

        # Compute intermediate points at intervals of delta_s
        num_points = int(distance // self.delta_s)

        for i in range(num_points):
            ratio = ((i+1) * self.delta_s) / distance
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
            if distance < self.delta_s / 2:
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
    
    def is_goal_reached(self,distance):
        if  not self.is_last_point:
            return False        
        elif distance < self.GOAL_REACHED_DIST:
            yaw_error = self.state[2] - self.static_path_plan[2,-1]
            if  abs(yaw_error) < self.GOAL_REACHED_YAW:
                return True
            else:
                return False
        else:
            return False
    
=======
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
>>>>>>> 8b92c94 (added mpc control and example, but still in working progress)

if __name__ == '__main__':
    mpc.main()
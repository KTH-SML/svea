#! /usr/bin/env python3

import numpy as np
import math
import rospy
import tf
from svea.models.bicycle import SimpleBicycleModel
from svea.states import VehicleState
from svea.simulators.sim_SVEA import SimSVEA
from svea.interfaces import LocalizationInterface
try:
    from svea_mocap.mocap import MotionCaptureInterface
except ImportError:
    pass
from svea.controllers.mpc import MPC_casadi
from svea.svea_managers.path_following_sveas import SVEAManagerMPC
from svea.data import TrajDataHandler, RVIZPathHandler
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray, PoseStamped
from svea.simulators.viz_utils import publish_pose_array


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)

class mpc_navigation:
    """
    This script implements a ROS node for controlling and simulating the SVEA vehicle autonomously.

    The node uses the default MPC algorithm to compute (plan) an optimal trajectory
    and control the vehicle to reach a specified target pose. The target pose is set interactively 
    using Foxglove publishing a geometry_msgs/PoseStamped message. 

    Key Features:
    - Generates a straight-line trajectory with intermediate points from the vehicle's current position 
    to the target pose, with a large distance between them to allow the MPC to optimally plan intermediate paths.
    - Supports both simulation and real-world operation, with integration with Motion Capture (Mocap).
    - Dynamically updates MPC parameters (final state weight matrix and prediction horizon) as the vehicle approaches the target for precise parking.
    - Publishes data like predicted paths, control targets, and velocities for visualization and debugging.

    The script is designed for optimal path planning and precise parking but does not handle obstacle avoidance. 
    It is ideal for scenarios where the vehicle must navigate to a predefined goal position with high accuracy.
    """
    def __init__(self,sim_dt,mpc):
        self.dt = sim_dt
        self.mpc = mpc

        ## ROS Parameters
        self.USE_RVIZ = load_param('~use_rviz', False)
        self.IS_SIM = load_param('~is_sim', False)
        self.STATE = load_param('~state', [-3, 0, 0, 0])    # [x,y,yaw,v] wrt map frame. Initial state for simulator.
        self.MPC_FREQ = load_param('~mpc_freq', 10)
        self.SVEA_MOCAP_NAME = load_param('~svea_mocap_name')
        self.DELTA_S = load_param('~delta_s', 5)            # static path discretization lenght
        self.mpc_config_ns = load_param('~mpc_config_ns')  
        self.initial_horizon = load_param(f'{self.mpc_config_ns}/prediction_horizon') 
        self.initial_Qf = load_param(f'{self.mpc_config_ns}/final_state_weight_matrix')  
        self.TARGET_SPEED = load_param('~target_speed', 0.5)  # Target speed. It is here for generalization, but not weighted in the optimization problem.

        ## MPC parameters 
        self.GOAL_REACHED_DIST = 0.2   # The distance threshold (in meters) within which the goal is considered reached.
        self.GOAL_REACHED_YAW = 0.2    # The yaw angle threshold (in radians) within which the goal orientation is considered reached.
        self.UPDATE_MPC_PARAM = True   # A flag indicating if the MPC parameters can be updated when the system is approaching the target.
        self.RESET_MPC_PARAM = False   # A flag indicating if the MPC parameters should be reset when the system is moving away from the target.
        self.predicted_state = None
        self.mpc_last_time = rospy.get_time()
        self.mpc_dt = 1.0 / self.MPC_FREQ 
        self.current_horizon = self.initial_horizon

        ## Static Planner parameters
        self.APPROACH_TARGET_THR = 5   # The distance threshold (in meters) to define when the system is "approaching" the target.
        self.NEW_REFERENCE_THR = 1     # The distance threshold (in meters) to update the next intermediate reference point. 
        self.goal_pose = None
        self.static_path_plan = np.empty((3, 0))
        self.current_index_static_plan = 0
        self.is_last_point = False

        ## Other parameters
        self.steering = 0
        self.velocity = 0
        self.state = []   

        ## Define the unitless steering biases for each SVEA.
        ## These values represent the measured steering actuations when the SVEA is not actually steering.
        self.unitless_steering_map = {
            "svea0": 28,
            "svea7": 7
        }
        if self.IS_SIM is False:
            svea_name = self.SVEA_MOCAP_NAME.lower()  # Ensure case-insensitivity  
            unitless_steering = self.unitless_steering_map.get(svea_name, 0)  # Default to 0 if not found
            PERC_TO_LLI_COEFF = 1.27
            MAX_STEERING_ANGLE = 40 * math.pi / 180
            steer_percent = unitless_steering / PERC_TO_LLI_COEFF
            self.steering_bias = (steer_percent / 100.0) * MAX_STEERING_ANGLE
        else:
            self.steering_bias = 0

        self.create_simulator_and_SVEAmanager()
        self.init_publishers()
        self.init_subscribers()

    def run(self):
        while self.keep_alive():
            self.spin()

    def keep_alive(self):
        return not rospy.is_shutdown()

    def spin(self):
        # Retrieve current state from SVEA localization
        state = self.svea.wait_for_state()
        self.state = [state.x, state.y, state.yaw, state.v]
        # If a static path plan has been computed, run the mpc.
        if self.static_path_plan.size > 0 :
            # If enough time has passed, run the MPC computation
            current_time = rospy.get_time()
            measured_dt = current_time - self.mpc_last_time
            if measured_dt >= self.mpc_dt :
                reference_trajectory, distance_to_next_point = self.get_mpc_current_reference()
                if self.is_last_point and distance_to_next_point <= self.APPROACH_TARGET_THR and self.UPDATE_MPC_PARAM:
                    # Update the prediction horizon and final state weight matrix only once when approaching target to achieve better parking.
                    new_Qf = np.array([70, 0, 0, 0,
                                        0, 70, 0, 0,
                                        0, 0, 20, 0,
                                        0, 0, 0, 0]).reshape((4, 4))
                    self.svea.controller.set_new_weight_matrix('Qf', new_Qf)
                    self.UPDATE_MPC_PARAM = False
                    self.RESET_MPC_PARAM = True  # Allow resetting when moving away

                elif self.is_last_point and distance_to_next_point > self.APPROACH_TARGET_THR and self.RESET_MPC_PARAM:
                    # Reset to initial values only once when moving away from target
                    self.current_horizon = self.initial_horizon
                    self.svea.controller.set_new_prediction_horizon(self.initial_horizon)
                    self.svea.controller.set_new_weight_matrix('Qf', self.initial_Qf)
                    self.UPDATE_MPC_PARAM = True  # Allow updating again when re-approaching
                    self.RESET_MPC_PARAM = False  # Prevent repeated resetting

                if  not self.is_goal_reached(distance_to_next_point):
                    # Run the MPC to compute control
                    steering_rate, acceleration = self.svea.controller.compute_control([self.state[0],self.state[1],self.state[2],self.velocity,self.steering], reference_trajectory)
                    self.steering += steering_rate * measured_dt
                    self.velocity += acceleration * measured_dt  
                    self.predicted_state = self.svea.controller.get_optimal_states()
                    # Publish the predicted path
                    self.publish_trajectory(self.predicted_state[0:3, :self.current_horizon+1],self.predicted_trajectory_pub)
                else:
                    # Stop the vehicle if the goal is reached
                    self.steering, self.velocity = 0, 0
                # Update the last time the MPC was computed
                self.mpc_last_time = current_time
            
        # Publish the latest control target and the estimated speed( from mocap or indoors loc. or outdoors loc.).
        self.publish_to_foxglove(self.steering, self.velocity, self.state[3])
        # Visualization data and send control
        self.svea.send_control(self.steering + self.steering_bias, self.velocity) 
        self.svea.visualize_data()
                
    def init_publishers(self):
        self.steering_pub = rospy.Publisher('/target_steering_angle', Float32, queue_size=1)
        self.velocity_pub = rospy.Publisher('/target_speed', Float32, queue_size=1)
        self.velocity_measured_pub = rospy.Publisher('/measured_speed', Float32, queue_size=1)   # estimated/measured speed
        self.predicted_trajectory_pub = rospy.Publisher('/predicted_path', PoseArray, queue_size=1)
        self.static_trajectory_pub = rospy.Publisher('/static_path', PoseArray, queue_size=1)

    def init_subscribers(self):
        self.mpc_target_sub = rospy.Subscriber('/mpc_target', PoseStamped, self.mpc_target_callback)

    def create_simulator_and_SVEAmanager(self):
        # initial state for simulator.
        state = VehicleState(*self.STATE)
        # Create simulators, models, managers, etc.
        if self.IS_SIM:

            # simulator need a model to simulate
            self.sim_model = SimpleBicycleModel(state)

            # start the simulator immediately, but paused
            self.simulator = SimSVEA(self.sim_model,
                                     dt=self.dt,
                                     run_lidar=True,
                                     start_paused=True,
                                     publish_odometry=True,
                                     publish_pose=True).start()

        # start the SVEA manager (needed for both sim and real world)
        self.svea = SVEAManagerMPC(localizer = LocalizationInterface if self.IS_SIM else MotionCaptureInterface,
                                controller = self.mpc,
                                data_handler=RVIZPathHandler if self.USE_RVIZ else TrajDataHandler,
                                vehicle_name='',
                                controller_config_path = self.mpc_config_ns)
        if not self.IS_SIM:
            self.svea.localizer.update_name(self.SVEA_MOCAP_NAME)

        self.svea.start(wait=True)

        # everything ready to go -> unpause simulator
        if self.IS_SIM:
            self.simulator.toggle_pause_simulation()

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
            target_speed_row = np.full((1, x_ref.shape[1]), self.TARGET_SPEED)                
            x_ref = np.concatenate((x_ref, target_speed_row), axis=0)
            return x_ref, distance_to_next_point

        else:
            distance_to_last_point = self.compute_distance(self.state,self.static_path_plan[:,-1])
            reference_state = self.static_path_plan[:,-1]
            x_ref = np.tile(reference_state, (self.initial_horizon+1, 1)).T
            target_speed_row = np.full((1, x_ref.shape[1]), self.TARGET_SPEED)
            x_ref = np.concatenate((x_ref, target_speed_row), axis=0)
            return x_ref, distance_to_last_point
    
    def mpc_target_callback(self, msg):
        """
        Callback function that sets a new goal position and calculates a trajectory.
        :param msg: PoseStamped message containing the goal position.
        """
        # Set the goal position and log the new goal
        self.goal_pose = msg
        rospy.loginfo(f"New goal position received: ({self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y})")
        # Compute trajectory (straight line from current position to goal)
        self.compute_trajectory()

    def get_yaw_from_pose(self,pose_stamped):
        """
        Extracts the yaw from a PoseStamped message.
        """
        # Convert the quaternion to Euler angles
        orientation = pose_stamped.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        
        # Use tf to convert quaternion to Euler angles
        euler = tf.transformations.euler_from_quaternion(quaternion)
        
        # Return the yaw
        return euler[2]  
    
    def compute_trajectory(self):
        """
        Compute a straight-line trajectory from the current position to the goal using DELTA_S,
        including the heading for each point, and publish the path.
        """
        if not self.goal_pose or not self.state:
            rospy.logwarn("Missing goal or current state for trajectory computation.")
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
        self.mpc_last_time = rospy.get_time()
        self.svea.controller.reset_parameters()

        # Calculate the straight-line trajectory between current state and goal position
        start_x, start_y = self.state[0], self.state[1]
        goal_x, goal_y = self.goal_pose.pose.position.x, self.goal_pose.pose.position.y
        distance = self.compute_distance([start_x, start_y], [goal_x, goal_y])
        goal_yaw = self.get_yaw_from_pose(self.goal_pose)

        # Compute intermediate points at intervals of DELTA_S
        num_points = int(distance // self.DELTA_S)

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

        # Publish the static path
        self.publish_trajectory(self.static_path_plan,self.static_trajectory_pub)
        
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
 
    def compute_distance(self,point1,point2):
        return np.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)   

    def publish_trajectory(self, points, publisher):
        pointx, pointy, pointyaw = [], [], []
        for i in range(points.shape[1]):  # points should be [3, N] where N is the horizon length
            pointx.append(points[0, i])  # x values
            pointy.append(points[1, i])  # y values
            pointyaw.append(points[2, i])  # yaw values        
        # Publish the trajectory as a PoseArray
        publish_pose_array(publisher, pointx, pointy, pointyaw)
       
if __name__ == '__main__':
    rospy.init_node('mpc_navigation')
    node = mpc_navigation(sim_dt = 0.01, mpc = MPC_casadi)
    node.run()
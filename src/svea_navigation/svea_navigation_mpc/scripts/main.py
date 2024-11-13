#! /usr/bin/env python3

import numpy as np
import math
import rospy
import tf
import yaml
from svea.models.bicycle import SimpleBicycleModel
from svea.states import VehicleState
from svea.simulators.sim_SVEA import SimSVEA
from svea.interfaces import LocalizationInterface
from svea_mocap.mocap import MotionCaptureInterface
from svea.controllers.mpc import MPC_casadi
from svea.svea_managers.path_following_sveas import SVEAManagerMPC
from svea.data import TrajDataHandler, RVIZPathHandler
from std_msgs.msg import Float32
from svea_navigation_mpc.srv import SetGoalPosition, SetGoalPositionResponse
from geometry_msgs.msg import PoseArray
from svea.simulators.viz_utils import publish_pose_array


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)

class main:
    """
    ROS Node for controlling and simulating the SVEA vehicle autonomously.
    """

    def __init__(self,sim_dt,mpc):
        self.dt = sim_dt
        self.mpc = mpc

        # ROS Parameters
        self.USE_RVIZ = load_param('~use_rviz', False)
        self.IS_SIM = load_param('~is_sim', False)
        self.USE_MOCAP = load_param('~use_mocap', False)
        self.STATE = load_param('~state', [-3, 0, 0, 0])    # [x,y,yaw,v] wrt map frame. Initial state for simulator.
        self.MPC_FREQ = load_param('~mpc_freq', 10)
        self.SVEA_MOCAP_NAME = load_param('~svea_mocap_name')
        self.DELTA_S = load_param('~delta_s', 5)            # static path discretization lenght
        self.mpc_config_file_path = load_param('~mpc_config_file_path')    
        self.GOAL_REACHED_DIST = 0.2   # meters
        self.GOAL_REACHED_YAW = 0.2   #  radians
        self.APPROACH_TARGET_THR = 2  # meters
        self.NEW_REFERENCE_THR = 2 # meters 
        self.UPDATE_MPC_PARAM = True   # Tracks if update can happen when approaching
        self.RESET_MPC_PARAM = False   # Tracks if reset can happen when moving away
        # Initialize optimal variables
        self.steering = 0
        self.velocity = 0
        self.predicted_state = None

        # Initialize other variables
        self.mpc_last_time = rospy.get_time()
        self.mpc_dt = 1.0 / self.MPC_FREQ 
        self.goal_pose = None
        self.state = []   
        self.static_path_plan = np.empty((3, 0))
        self.current_index_static_plan = 0
        self.is_last_point = False

        # Load parameters from the YAML file 
        with open(self.mpc_config_file_path, 'r') as file:
            config = yaml.safe_load(file)
        # Initialize parameters from YAML file
        self.initial_horizon = config['prediction_horizon']  
        self.initial_Qf = config['final_state_weight_matrix']  
        self.current_horizon = self.initial_horizon
        if self.IS_SIM is False:
            # add steering bias of svea0
            unitless_steering = 28
            PERC_TO_LLI_COEFF = 1.27
            MAX_STEERING_ANGLE = 40 * math.pi / 180
            steer_percent = unitless_steering / PERC_TO_LLI_COEFF
            self.steering_bias = (steer_percent / 100.0) * MAX_STEERING_ANGLE
        else:
            self.steering_bias = 0

        self.create_simulator_and_SVEAmanager()
        self.init_publishers()
        self.init_services()

    def run(self):
        while self.keep_alive():
            self.spin()

    def keep_alive(self):
        return not rospy.is_shutdown()

    def spin(self):
        # Retrieve current state from SVEA localization
        state = self.svea.wait_for_state()
        #print(state)
        self.state = [state.x, state.y, state.yaw, state.v]
        #print("v localization",self.state[3])
        #actuated_steering = self.svea.actuation.ctrl_actuated_log[-1].steering
        # If a static path plan has been computed, run the mpc.
        if self.static_path_plan.size > 0 :
            # If enough time has passed, run the MPC computation
            current_time = rospy.get_time()
            measured_dt = current_time - self.mpc_last_time
            if measured_dt >= self.mpc_dt :
                reference_trajectory, distance_to_next_point = self.get_mpc_current_reference()
                if self.is_last_point and distance_to_next_point <= self.APPROACH_TARGET_THR and self.UPDATE_MPC_PARAM:
                    # Update the prediction horizon and final state weight matrix only once when approaching target
                    new_horizon = 5
                  #  self.current_horizon = new_horizon
                    new_Qf = np.array([70, 0, 0, 0,
                                        0, 70, 0, 0,
                                        0, 0, 20, 0,
                                        0, 0, 0, 0]).reshape((4, 4))
                   # self.svea.controller.set_new_prediction_horizon(new_horizon)
                    self.svea.controller.update_weight_matrices('Qf', new_Qf)
                    self.UPDATE_MPC_PARAM = False
                    self.RESET_MPC_PARAM = True  # Allow resetting when moving away

                elif self.is_last_point and distance_to_next_point > self.APPROACH_TARGET_THR and self.RESET_MPC_PARAM:
                    # Reset to initial values only once when moving away from target
                    self.current_horizon = self.initial_horizon
                    self.svea.controller.set_new_prediction_horizon(self.initial_horizon)
                    self.svea.controller.update_weight_matrices('Qf', self.initial_Qf)
                    self.UPDATE_MPC_PARAM = True  # Allow updating again when re-approaching
                    self.RESET_MPC_PARAM = False  # Prevent repeated resetting

                if  not self.is_goal_reached(distance_to_next_point):
                    # Run the MPC to compute control
                    steering_rate, acceleration = self.svea.controller.compute_control([self.state[0],self.state[1],self.state[2],self.velocity,self.steering], reference_trajectory)
                    self.steering += steering_rate * measured_dt
                    self.velocity += acceleration * measured_dt  
                    self.predicted_state = self.svea.controller.get_optimal_states()
                    #print("velocity command", self.velocity)
                    #control = self.svea.controller.get_optimal_control()
                    #print("control command", control)
                    # Publish the predicted path
                    self.publish_trajectory(self.predicted_state[0:3, :self.current_horizon+1],self.predicted_trajectory_pub)
                else:
                    # Stop the vehicle if the goal is reached
                    self.steering, self.velocity = 0, 0
                    #print("GOAL ACHIEVED",self.state)
                # Update the last time the MPC was computed
                self.mpc_last_time = current_time
            
        # Publish the latest control target and the estimated(mocap) / measured(in/out loc.) speed.
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

    def init_services(self):
        self.goal_service = rospy.Service('/set_goal_position', SetGoalPosition, self.handle_set_goal_position)

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
        self.svea = SVEAManagerMPC(localizer = MotionCaptureInterface if self.USE_MOCAP else LocalizationInterface,
                                controller = self.mpc,
                                data_handler=RVIZPathHandler if self.USE_RVIZ else TrajDataHandler,
                                vehicle_name='',
                                controller_config_path = self.mpc_config_file_path)
        if self.USE_MOCAP:
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
            (shape: [3, N+1]) and distance_to_next_point is the distance to the next or last reference point.
        """
        if self.is_last_point is False:
            distance_to_next_point = self.compute_distance(self.state,self.static_path_plan[:,self.current_index_static_plan])
            if self.current_index_static_plan == self.static_path_plan.shape[1] - 1:
                self.is_last_point = True
            if distance_to_next_point < self.NEW_REFERENCE_THR and not self.is_last_point:
                self.current_index_static_plan += 1  
            reference_state = self.static_path_plan[:,self.current_index_static_plan]
            x_ref = np.tile(reference_state, (self.initial_horizon+1, 1)).T     
            return x_ref, distance_to_next_point

        else:
            distance_to_last_point = self.compute_distance(self.state,self.static_path_plan[:,-1])
            reference_state = self.static_path_plan[:,-1]
            x_ref = np.tile(reference_state, (self.initial_horizon+1, 1)).T
            return x_ref, distance_to_last_point
    
    def handle_set_goal_position(self, req):
        """
        Service handler that sets a new goal position and calculates a trajectory.
        :param req: SetGoalPositionRequest containing the goal PoseStamped.
        :return: SetGoalPositionResponse
        """
        self.goal_pose = req.goal_pose
        rospy.loginfo(f"New goal position received: ({self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y})")

        # Compute trajectory (straight line from current position to goal)
        self.compute_trajectory()

        # Respond with success
        res = SetGoalPositionResponse()
        res.success = True
        res.message = "Goal position set successfully and trajectory computed."
        return res

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
        # TODO: implemet reset function within mpc class for effeciency and readability.
        self.current_horizon = self.initial_horizon
        self.svea.controller.set_new_prediction_horizon(self.initial_horizon)
        self.svea.controller.update_weight_matrices('Qf',np.array(self.initial_Qf).reshape((4, 4)))

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
    rospy.init_node('main')
    node = main(sim_dt = 0.01, mpc = MPC_casadi)
    node.run()
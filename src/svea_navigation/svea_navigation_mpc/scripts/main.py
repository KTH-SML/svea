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
from mpc_casadi import MPC_casadi
from svea.svea_managers.svea_archetypes import SVEAManager
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
        self.IS_SIM = load_param('~is_sim', False)
        self.USE_MOCAP = load_param('~use_mocap', False)
        self.STATE = load_param('~state', [3, 0, 0, 0])    # [x,y,yaw,v] wrt map frame. Initial state for simulator.
        self.MPC_FREQ = load_param('~mpc_freq', 10)
        self.MOCAP_NAME = load_param('~mocap_name')
        self.GOAL_REACHED_DIST = 0.2   # meters
        self.GOAL_REACHED_YAW = 0.2   #  radians
        self.REDUCE_PREDICTION_HORIZON_THR = 0.0  # meters
        self.NEW_REFERENCE_THR = 0.5 # meters 
        self.DELTA_S = 2   # TODO: get it from launch file
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

        # TODO: import yaml here and set mpc params from here. set path in launch file.
        config_path = '/svea_ws/src/svea_navigation/svea_navigation_mpc/params/mpc_params.yaml'
        # Load parameters from the YAML file 
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        # Initialize parameters from YAML file
        self.N = config['prediction_horizon']  
        self.new_horizon = self.N

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
        self.state = [state.x, state.y, state.yaw, state.v]
        #print("v localization",state[3])
        # If a static path plan has been computed, run the mpc.
        if self.static_path_plan.size > 0 :
            # If enough time has passed, run the MPC computation
            current_time = rospy.get_time()
            if current_time - self.mpc_last_time >= self.mpc_dt:
                reference_trajectory, distance_to_next_point = self.get_mpc_current_reference()
                if distance_to_next_point < self.REDUCE_PREDICTION_HORIZON_THR:
                    self.new_horizon = math.ceil(5)
                    #print(self.new_horizon)
                    self.svea.controller.set_new_prediction_horizon(self.new_horizon)
                if  not self.is_goal_reached(distance_to_next_point):
                    # Run the MPC to compute control
                    steering_rate, acceleration = self.svea.controller.compute_control([self.state[0],self.state[1],self.state[2],self.velocity,self.steering], reference_trajectory)
                    self.steering += steering_rate * self.mpc_dt
                    self.velocity += acceleration * self.mpc_dt  
                    self.predicted_state = self.svea.controller.get_optimal_states()
                    #print("velocity command", self.velocity)
                    #control = self.svea.controller.get_optimal_control()
                    #print("control command", control)
                    # Publish the predicted path
                    self.publish_trajectory(self.predicted_state[0:3, :self.new_horizon+1],self.predicted_trajectory_pub)
                else:
                    # Stop the vehicle if the goal is reached
                    self.steering, self.velocity = 0, 0
                    #print("GOAL ACHIEVED",self.state)
                # Update the last time the MPC was computed
                self.mpc_last_time = current_time
            
        # Publish the latest control, whether newly computed or the last one
        self.publish_control(self.steering, self.velocity)
        # Visualization data and send control
        self.svea.send_control(self.steering, self.velocity) 
        self.svea.visualize_data()
                
    def init_publishers(self):
        self.steering_pub = rospy.Publisher('/nav_steering_angle', Float32, queue_size=1)
        self.velocity_pub = rospy.Publisher('/nav_vehicle_velocity', Float32, queue_size=1)
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
        localizer = MotionCaptureInterface if self.USE_MOCAP else LocalizationInterface
        print(localizer)
        self.svea = SVEAManager(localizer=localizer,
                                controller = self.mpc,
                                data_handler=RVIZPathHandler if self.USE_RVIZ else TrajDataHandler)
        if self.USE_MOCAP:
            self.svea.localizer.update_name(self.MOCAP_NAME)

        self.svea.start(wait=True)

        # everything ready to go -> unpause simulator
        if self.IS_SIM:
            self.simulator.toggle_pause_simulation()

    def publish_control(self,steering,velocity):
        self.steering_pub.publish(steering)
        self.velocity_pub.publish(velocity)

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
            x_ref = np.tile(reference_state, (self.N+1, 1)).T     
            return x_ref, distance_to_next_point

        else:
            distance_to_last_point = self.compute_distance(self.state,self.static_path_plan[:,-1])
            reference_state = self.static_path_plan[:,-1]
            x_ref = np.tile(reference_state, (self.N+1, 1)).T
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
        # Reset previous trjectory related variables
        self.current_index_static_plan = 0
        self.is_last_point = False
        self.static_path_plan = np.empty((3, 0))

        # Calculate the straight-line trajectory between current state and goal position
        start_x, start_y = self.state[0], self.state[1]
        goal_x, goal_y = self.goal_pose.pose.position.x, self.goal_pose.pose.position.y
        distance = self.compute_distance([start_x, start_y], [goal_x, goal_y])
        goal_yaw = self.get_yaw_from_pose(self.goal_pose)

        # Compute intermediate points at intervals of DELTA_S
        num_points = int(distance // self.DELTA_S)
        print(num_points)
        for i in range(num_points):
            ratio = ((i+1) * self.DELTA_S) / distance
            x = start_x + ratio * (goal_x - start_x)
            y = start_y + ratio * (goal_y - start_y)
            
            # Calculate the heading for this point
            heading = math.atan2(goal_y - start_y, goal_x - start_x)
            
            # Stack the computed point as a new column in the array
            new_point = np.array([[x], [y], [heading]])
            self.static_path_plan = np.hstack((self.static_path_plan, new_point))

        last_x = goal_x
        last_y = goal_y         
        # Append the last point
        new_point = np.array([[last_x], [last_y], [goal_yaw]])
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
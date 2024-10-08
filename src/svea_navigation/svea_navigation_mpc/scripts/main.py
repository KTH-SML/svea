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
        self.STATE = load_param('~state', [3, 0, 0, 0])    # [x,y,yaw,v] wrt map frame. Initial state for simulator.
        self.MPC_FREQ = load_param('~mpc_freq', 10)
        self.GOAL_REACHED_DIST = 0.2   # meters
        self.GOAL_REACHED_YAW = 0.2   #  radians
        self.REDUCE_PREDICTION_HORIZON_THR = 0.3  # meters
        self.delta_s = 1
        # Initialize optimal variables
        self.steering = 0
        self.velocity = 0
        self.predicted_state = None
        # Initialize other variables
        self.mpc_last_time = rospy.get_time()
        self.mpc_dt = 1.0 / self.MPC_FREQ 
        self.goal_pose = None
        self.state = []

        # TODO: import yaml here and set mpc params from here. set path in launch file.
        config_path = '/svea_ws/src/svea_navigation/svea_navigation_mpc/params/mpc_params.yaml'
        # Load parameters from the YAML file 
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        # Initialize parameters from YAML file
        self.new_horizon = config['prediction_horizon']  

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
        # If enough time has passed, run the MPC computation
        current_time = rospy.get_time()
        if current_time - self.mpc_last_time >= self.mpc_dt:
            reference_trajectory = self.get_reference_trajectory()
            distance_to_goal = self.compute_distance(self.state, reference_trajectory[0:2, -1])
            yaw_to_goal = self.state[2] - reference_trajectory[2, -1]
            if distance_to_goal < self.REDUCE_PREDICTION_HORIZON_THR:
                self.new_horizon = math.ceil(5)
                #print(self.new_horizon)
                self.svea.controller.set_new_prediction_horizon(self.new_horizon)
            if  not self.is_goal_reached(distance_to_goal,yaw_to_goal):
                # Run the MPC to compute control
                steering_rate, acceleration = self.svea.controller.compute_control([self.state[0],self.state[1],self.state[2],self.velocity,self.steering], reference_trajectory)
                self.steering += steering_rate * self.mpc_dt
                self.velocity += acceleration * self.mpc_dt  
                self.predicted_state = self.svea.controller.get_optimal_states()
                #print("velocity command", self.velocity)
                #control = self.svea.controller.get_optimal_control()
                #print("control command", control)
                # Publish the predicted path
                self.publish_predicted_path(self.predicted_state[0:3, :self.new_horizon+1])
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
        self.svea = SVEAManager(LocalizationInterface,
                                    self.mpc,
                                    data_handler=RVIZPathHandler if self.USE_RVIZ else TrajDataHandler)
        self.svea.start(wait=True)

        # everything ready to go -> unpause simulator
        if self.IS_SIM:
            self.simulator.toggle_pause_simulation()

    def publish_control(self,steering,velocity):
        self.steering_pub.publish(steering)
        self.velocity_pub.publish(velocity)

    def get_reference_trajectory(self):
        reference_state = [self.STATE[0] + 1,self.STATE[1],self.STATE[2]+0,self.STATE[3]]
        x_ref = np.tile(reference_state, (11, 1)).T 
        return x_ref
    
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
        Compute a straight-line trajectory from the current position to the goal using delta_s,
        including the heading for each point, and publish the path.
        """
        if not self.goal_pose or not self.state:
            rospy.logwarn("Missing goal or current state for trajectory computation.")
            return

        # Calculate the straight-line trajectory between current state and goal position
        start_x, start_y = self.state[0], self.state[1]
        goal_x, goal_y = self.goal_pose.pose.position.x, self.goal_pose.pose.position.y
        distance = self.compute_distance([start_x, start_y], [goal_x, goal_y])
        goal_yaw = self.get_yaw_from_pose(self.goal_pose)

        # Compute intermediate points at intervals of delta_s
        num_points = int(distance // self.delta_s)
        trajectory = np.zeros((3, num_points + 1))  # shape [3, N]

        for i in range(num_points):
            ratio = (i * self.delta_s) / distance
            x = start_x + ratio * (goal_x - start_x)
            y = start_y + ratio * (goal_y - start_y)
            
            # Calculate the heading for this point
            heading = math.atan2(goal_y - start_y, goal_x - start_x)
            
            trajectory[0, i] = x  # x values
            trajectory[1, i] = y  # y values
            trajectory[2, i] = heading  # yaw values

        # Handle any leftover distance
        if distance % self.delta_s > 0:
            # Calculate last point with the goal heading
            last_x = goal_x
            last_y = goal_y
            heading = math.atan2(goal_y - start_y, goal_x - start_x)
            
            trajectory[0, num_points] = last_x
            trajectory[1, num_points] = last_y
            trajectory[2, num_points] = goal_yaw

        # Publish the predicted path
        self.publish_predicted_path(trajectory)

    def _visualize_trajectory(self, trajectory):
        """Use svea manager handler to visualize the computed trajectory."""
        traj_x = [point[0] for point in trajectory]
        traj_y = [point[1] for point in trajectory]
        self.svea.data_handler.update_traj(traj_x, traj_y)
        
    def is_goal_reached(self,distance,yaw_error):
        if distance < self.GOAL_REACHED_DIST and abs(yaw_error) < self.GOAL_REACHED_YAW:
            return True
        else:
            return False
 
    def compute_distance(self,point1,point2):
        return np.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)   

    def publish_predicted_path(self, points):
        pointx, pointy, pointyaw = [], [], []
        for i in range(points.shape[1]):  # points should be [3, N] where N is the horizon length
            pointx.append(points[0, i])  # x values
            pointy.append(points[1, i])  # y values
            pointyaw.append(points[2, i])  # yaw values        
        # Publish the trajectory as a PoseArray
        publish_pose_array(self.predicted_trajectory_pub, pointx, pointy, pointyaw)

if __name__ == '__main__':
    rospy.init_node('main')
    node = main(sim_dt = 0.01, mpc = MPC_casadi)
    node.run()
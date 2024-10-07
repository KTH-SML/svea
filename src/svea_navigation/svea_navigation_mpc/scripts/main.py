#! /usr/bin/env python3

import numpy as np
import math
import rospy
import yaml
from svea.models.bicycle import SimpleBicycleModel
from svea.states import VehicleState
from svea.simulators.sim_SVEA import SimSVEA
from svea.interfaces import LocalizationInterface
from mpc_casadi import MPC_casadi
from svea.svea_managers.svea_archetypes import SVEAManager
from svea.data import TrajDataHandler, RVIZPathHandler
from std_msgs.msg import Float32
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
        self.REDUCE_PREDICTION_HORIZON_THR = 1  # meters
        # Initialize control variables
        self.steering = 0
        self.velocity = 0
        self.predicted_state = None
        # Initialize other variables
        self.mpc_last_time = rospy.get_time()
        self.mpc_dt = 1.0 / self.MPC_FREQ  

        # TODO: import yaml here and set mpc params from here. set path in launch file.
        config_path = '/svea_ws/src/svea_navigation/svea_navigation_mpc/params/mpc_params.yaml'
        # Load parameters from the YAML file 
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        # Initialize parameters from YAML file
        self.new_horizon = config['prediction_horizon']  

        self.create_simulator_and_SVEAmanager()
        self.init_publishers()

    def run(self):
        while self.keep_alive():
            self.spin()

    def keep_alive(self):
        return not rospy.is_shutdown()

    def spin(self):
        # Retrieve current state from SVEA localization
        state = self.svea.wait_for_state()
        state = [state.x, state.y, state.yaw, state.v]
        #print("v localization",state[3])
        # If enough time has passed, run the MPC computation
        current_time = rospy.get_time()
        if current_time - self.mpc_last_time >= self.mpc_dt:
            reference_trajectory = self.get_reference_trajectory()
            distance_to_goal = self.compute_distance(state, reference_trajectory[0:2, -1])
            yaw_to_goal = state[2] - reference_trajectory[2, -1]
            if distance_to_goal < self.REDUCE_PREDICTION_HORIZON_THR:
                self.new_horizon = math.ceil(3+ (distance_to_goal / 0.5))
                #print(self.new_horizon)
                self.svea.controller.set_new_prediction_horizon(self.new_horizon)
            if  not self.is_goal_reached(distance_to_goal,yaw_to_goal):
                # Run the MPC to compute control
                self.steering, self.velocity = self.svea.controller.compute_control([state[0],state[1],state[2],self.velocity], reference_trajectory)  
                self.predicted_state = self.svea.controller.get_optimal_states()
                #print("velocity command", self.velocity)
                #control = self.svea.controller.get_optimal_control()
                #print("control command", control)
                # Publish the predicted path
                self.publish_predicted_path(self.predicted_state[0:3, :self.new_horizon+1])
            else:
                # Stop the vehicle if the goal is reached
                self.steering, self.velocity = 0, 0
                print("GOAL ACHIEVED",state)
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
        reference_state = [self.STATE[0] + 3,self.STATE[1],self.STATE[2]+3.14,self.STATE[3]]
        x_ref = np.tile(reference_state, (11, 1)).T 
        return x_ref
    
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
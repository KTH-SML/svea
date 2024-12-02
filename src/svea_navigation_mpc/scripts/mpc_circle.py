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
from geometry_msgs.msg import PoseArray
from svea.simulators.viz_utils import publish_pose_array


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)

class mpc_navigation:
    """
    ROS Node for controlling and simulating the SVEA vehicle autonomously.
    """

    def __init__(self,sim_dt,mpc):
        self.dt = sim_dt
        self.mpc = mpc

        ## ROS Parameters
        self.USE_RVIZ = load_param('~use_rviz', False)
        self.IS_SIM = load_param('~is_sim', False)
        self.STATE = load_param('~state', [2, 0, 2, 0])    # [x,y,yaw,v] wrt map frame. Initial state for simulator.
        self.MPC_FREQ = load_param('~mpc_freq', 10)
        self.SVEA_MOCAP_NAME = load_param('~svea_mocap_name')
        self.DELTA_THETA = load_param('~delta_theta', 0.1)  # Circle discretization step in radians
        self.CIRCLE_RADIUS = load_param('~circle_radius', 5) # Radius of the static circle path
        self.CIRCLE_CENTER_X = load_param('~circle_center_x', 0) # X coordinate of the circle center
        self.CIRCLE_CENTER_Y = load_param('~circle_center_y', 0) # Y coordinate of the circle center
        self.mpc_config_ns = load_param('~mpc_config_ns')  
        self.initial_horizon = load_param(f'{self.mpc_config_ns}/prediction_horizon')  

        ## MPC parameters 
        self.predicted_state = None
        self.mpc_last_time = 0
        self.mpc_dt = 1.0 / self.MPC_FREQ 
        self.current_horizon = self.initial_horizon

        ## Static Planner parameters
        self.NEW_REFERENCE_THR = 0.2     # The distance threshold (in meters) to update the next intermediate reference point. 
        self.goal_pose = None
        self.static_path_plan = np.empty((3, 0))
        self.current_index_static_plan = 0
        self.is_last_point = False

        ## Other parameters
        self.steering = 0
        self.velocity = 0
        self.state = []   

        if self.IS_SIM is False:
            # add steering bias of sveas. {(svea0:28),(svea7:7)}
            unitless_steering = 7     
            PERC_TO_LLI_COEFF = 1.27
            MAX_STEERING_ANGLE = 40 * math.pi / 180
            steer_percent = unitless_steering / PERC_TO_LLI_COEFF
            self.steering_bias = (steer_percent / 100.0) * MAX_STEERING_ANGLE
        else:
            self.steering_bias = 0

        self.create_simulator_and_SVEAmanager()
        self.init_publishers()

    def run(self):
        while self.keep_alive():
            self.spin()

    def keep_alive(self):
        return not rospy.is_shutdown()

    def spin(self):
        #print(self.static_path_plan)
        # Generate static circle path once.
        if self.static_path_plan.size == 0:
            self.generate_static_circle_path()

        # Retrieve current state from SVEA localization
        state = self.svea.wait_for_state()
        self.state = [state.x, state.y, state.yaw, state.v]
        # Run the MPC if the static path plan is available
        if self.static_path_plan.size > 0:
            current_time = rospy.get_time()
            measured_dt = current_time - self.mpc_last_time
            if measured_dt >= self.mpc_dt:
                reference_trajectory = self.get_mpc_current_reference()
                # Run the MPC to compute control
                steering_rate, acceleration = self.svea.controller.compute_control(
                    [self.state[0], self.state[1], self.state[2], self.velocity, self.steering], reference_trajectory
                )
                self.steering += steering_rate * measured_dt
                self.velocity += acceleration * measured_dt
                self.predicted_state = self.svea.controller.get_optimal_states()
                # Publish the predicted path
                self.publish_trajectory(self.predicted_state[0:3, :self.current_horizon+1], self.predicted_trajectory_pub)
                self.mpc_last_time = current_time

        # Publish the latest control target and the estimated(mocap) / measured(in/out loc.) speed.
        self.publish_to_foxglove(self.steering, self.velocity, self.state[3])
        # Visualization data and send control
        self.svea.send_control(self.steering + self.steering_bias, self.velocity) 
        self.svea.visualize_data()

    def init_publishers(self):
        self.steering_pub = rospy.Publisher('/target_steering_angle', Float32, queue_size=1)
        self.velocity_pub = rospy.Publisher('/target_speed', Float32, queue_size=1)
        self.velocity_measured_pub = rospy.Publisher('/measured_speed', Float32, queue_size=1)
        self.predicted_trajectory_pub = rospy.Publisher('/predicted_path', PoseArray, queue_size=1)
        self.static_trajectory_pub = rospy.Publisher('/static_path', PoseArray, queue_size=1)

    def generate_static_circle_path(self):
        """
        Generates a circular path based on the specified radius, center, and discretization step (DELTA_THETA).
        """
        rospy.sleep(2.0)
        theta_values = np.arange(0, 2 * math.pi, self.DELTA_THETA)
        x_values = self.CIRCLE_CENTER_X + self.CIRCLE_RADIUS * np.cos(theta_values)
        y_values = self.CIRCLE_CENTER_Y + self.CIRCLE_RADIUS * np.sin(theta_values)
        yaw_values = np.arctan2(np.diff(y_values, append=y_values[0]), np.diff(x_values, append=x_values[0]))
        self.static_path_plan = np.vstack((x_values, y_values, yaw_values))
        self.mpc_last_time = rospy.get_time()
        # Publish the static path
        self.publish_trajectory(self.static_path_plan, self.static_trajectory_pub)

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

    def get_mpc_current_reference(self):
        """
        Retrieves the current reference state for the MPC based on the SVEA's current position.
        Updates the current index in the static path plan if the robot is close to the next point.
        """
        distance_to_next_point = self.compute_distance(self.state, self.static_path_plan[:, self.current_index_static_plan])
        if distance_to_next_point < self.NEW_REFERENCE_THR:
            self.current_index_static_plan = (self.current_index_static_plan + 1) % self.static_path_plan.shape[1]
        reference_state = self.static_path_plan[:, self.current_index_static_plan]
        x_ref = np.tile(reference_state, (self.initial_horizon + 1, 1)).T     
        return x_ref

    def compute_distance(self, point1, point2):
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)   

    def publish_trajectory(self, points, publisher):
        pointx, pointy, pointyaw = [], [], []
        for i in range(points.shape[1]):  # points should be [3, N] where N is the horizon length
            pointx.append(points[0, i])  # x values
            pointy.append(points[1, i])  # y values
            pointyaw.append(points[2, i])  # yaw values        
        # Publish the trajectory as a PoseArray
        publish_pose_array(publisher, pointx, pointy, pointyaw)

    def publish_to_foxglove(self,target_steering,target_speed,measured_speed):
        self.steering_pub.publish(target_steering)
        self.velocity_pub.publish(target_speed)
        self.velocity_measured_pub.publish(measured_speed)
       
if __name__ == '__main__':
    rospy.init_node('mpc_navigation')
    node = mpc_navigation(sim_dt=0.01, mpc=MPC_casadi)
    node.run()

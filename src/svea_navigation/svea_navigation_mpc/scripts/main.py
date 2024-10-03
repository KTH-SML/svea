#! /usr/bin/env python3

import numpy as np
import rospy
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
        # Initialize control variables
        self.steering = 0
        self.velocity = 0
        self.predicted_state = None
        # Initialize a variable to track time for MPC computation
        self.mpc_last_time = rospy.get_time()
        self.mpc_dt = 1.0 / self.MPC_FREQ  

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
        # If enough time has passed, run the MPC computation
        current_time = rospy.get_time()
        if current_time - self.mpc_last_time >= self.mpc_dt:
            reference_trajectory = self.get_reference_trajectory()
            if not self.is_goal_reached(state, reference_trajectory[0:2, -1]):
                # Run the MPC to compute control
                self.steering, self.velocity, self.predicted_state = self.svea.controller.compute_control(state, reference_trajectory)       
                # Publish the predicted path
                self.publish_predicted_path(self.predicted_state[0:3, :])
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
        reference_state = [self.STATE[0] + 2,self.STATE[1],self.STATE[2]+np.pi,self.STATE[3]]
        x_ref = np.tile(reference_state, (26, 1)).T 
        return x_ref
 
    def is_goal_reached(self,state,goal):
        return False #(state[0]-goal[0])**2 + (state[1]-goal[1])**2 <= 0.01    

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
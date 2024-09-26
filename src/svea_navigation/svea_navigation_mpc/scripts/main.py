#! /usr/bin/env python3

import numpy as np
import rospy
from svea.models.bicycle import SimpleBicycleModel
from svea.states import VehicleState
from svea.simulators.sim_SVEA import SimSVEA
from svea.interfaces import LocalizationInterface
from base_local_planner_controller import BaseLocalPlannerController
from svea.svea_managers.svea_archetypes import SVEAManager
from svea.data import TrajDataHandler, RVIZPathHandler
from std_msgs.msg import Float32


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)

class main:
    """
    ROS Node for controlling and simulating the SVEA vehicle autonomously.
    """

    DELTA_TIME = 0.01

    def __init__(self):
        rospy.init_node('main')

        # Parameters
        self.USE_RVIZ = load_param('~use_rviz', False)
        self.IS_SIM = load_param('~is_sim', False)
        self.STATE = load_param('~state', [3, 0, 0, 0])    # [x,y,yaw,v] wrt map frame.

        # initial state for simulator.
        state = VehicleState(*self.STATE)

        self.steering_pub = rospy.Publisher('/nav_steering_angle', Float32, queue_size=1)
        self.velocity_pub = rospy.Publisher('/nav_vehicle_velocity', Float32, queue_size=1)

        # Create simulators, models, managers, etc.
        if self.IS_SIM:

            # simulator need a model to simulate
            self.sim_model = SimpleBicycleModel(state)

            # start the simulator immediately, but paused
            self.simulator = SimSVEA(self.sim_model,
                                     dt=self.DELTA_TIME,
                                     run_lidar=True,
                                     start_paused=True,
                                     publish_odometry=True,
                                     publish_pose=True).start()

        # start the SVEA manager (needed for both sim and real world)
        self.svea = SVEAManager(LocalizationInterface,
                                    BaseLocalPlannerController,
                                    data_handler=RVIZPathHandler if self.USE_RVIZ else TrajDataHandler)
        self.svea.start(wait=True)

        # everything ready to go -> unpause simulator
        if self.IS_SIM:
            self.simulator.toggle_pause_simulation()

    def run(self):
        while self.keep_alive():
            self.spin()

    def keep_alive(self):
        return not rospy.is_shutdown()

    def spin(self):
        state = self.svea.wait_for_state()                  # limit the rate of main loop by waiting for state
        steering, velocity = self.svea.compute_control()
        self.publish_control(steering,velocity)
        self.svea.send_control(steering, velocity)
        self.svea.visualize_data()
        
    def publish_control(self,steering,velocity):
        self.steering_pub.publish(steering)
        self.velocity_pub.publish(velocity)


if __name__ == '__main__':
    main().run()
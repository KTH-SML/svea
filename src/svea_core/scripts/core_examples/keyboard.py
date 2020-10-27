#!/usr/bin/env python

import math
import rospy

from geometry_msgs.msg import Twist

from svea.svea_managers.svea_archetypes import SVEAManager
from svea.states import VehicleState
from svea.localizers import LocalizationInterface
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA


## SIMULATION PARAMS ##########################################################
vehicle_name = "SVEA"
init_state = [0.0, 0.0, 0.0, 0.0] #[x, y, yaw, v], units: [m, m, rad, m/s]
init_state = VehicleState(*init_state)
dt = 0.01

# animate results
show_animation = True
###############################################################################


class KeyboardDummyController(object):
    def __init__(self, vehicle_name=''):
        self.steering = 0
        self.velocity = 0
        rospy.Subscriber("/key_vel", Twist, self._update_key_teleop)

    def _update_key_teleop(self, key_msg):
        self.steering = math.radians(key_msg.angular.z)
        self.velocity = key_msg.linear.x

    def compute_control(self, state):
        return self.steering, self.velocity

def param_init():
    # grab parameters from launch-file
    is_sim_param = rospy.search_param('is_sim')
    is_sim = rospy.get_param(is_sim_param, True)
    return is_sim

def main():
    rospy.init_node('SVEA_keyboard')
    is_sim = param_init()

    if is_sim:
        # start the simulation
        model_for_sim = SimpleBicycleModel(init_state)
        simulator = SimSVEA(vehicle_name, model_for_sim,
                            dt=dt, start_paused=True).start()

    # start manager of the simulated SVEA
    svea = SVEAManager(vehicle_name,
                       LocalizationInterface,
                       KeyboardDummyController)
    svea.start(wait=True)

    if is_sim:
        # start simulation
        simulator.toggle_pause_simulation()

    # control loop
    while not rospy.is_shutdown():
        svea.wait_for_state() # loop dictated by when state data comes in

        # pass keyboard commands to actuation interface
        steering, velocity = svea.compute_control() # just returns raw keyboard
        svea.send_control(steering, velocity)

        # visualize data
        if show_animation:
            svea.visualize_data()

    rospy.spin()

if __name__ == '__main__':
    main()

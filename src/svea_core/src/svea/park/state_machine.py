#! /usr/bin/env python3

import rospy
from rospy import Publisher, Rate

from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

from svea.park.vehicle_manager import VehicleManager
from svea.park.drive_state import DriveState


class StateMachine:

    def __init__(self):

        ## Initialize node

        rospy.init_node('state_machine')

        self.rate = rospy.Rate(10) # TODO: Check if this is the right rate

        # Auxiliary objects
        self.vehicle_manager = VehicleManager() # Keep track of global information about the vehicle      

        # Initialize objects of state machine states
        self.drive_state = DriveState(controller=self.vehicle_manager.drive_controller)

    def run(self):
        rospy.loginfo("Starting state machine")
        while not rospy.is_shutdown():
            self.spin()

    def spin(self):

        self.vehicle_manager = self.drive_state.run(self.vehicle_manager) 

        self.rate.sleep()


if __name__ == '__main__':

    StateMachine().run()
#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped

from svea_msgs.msg import VehicleState as VehicleStateMsg
from svea.states import VehicleState
from svea.data import RVIZPathHandler


class state_publisher:
    """
    A quick and easy way to get vehicle states.

    Launch the corresponding launch file and this node will continuously print
    out the vehicle state (coordinates). Useful when finding coordinates in a
    map.
    """

    SPIN_RATE = 10

    def __init__(self):

        ## Initialize node

        rospy.init_node('state_publisher')

        ## Create node resources

        # spin rate
        self.rate = rospy.Rate(self.SPIN_RATE)

        # Visualization tool
        self.rviz = RVIZPathHandler()

        # Vehicle state
        self.state = VehicleState()
        self.state.state_msg = rospy.wait_for_message('state', VehicleStateMsg)
        rospy.Subscriber(
            "state",
            VehicleStateMsg,
            lambda msg: setattr(self.state, 'state_msg', msg)
        )

        # Clicked point listener
        self.sub_clicked_point = rospy.Subscriber(
            '/clicked_point',
            PointStamped,
            lambda msg: rospy.loginfo('Clikcked point: [%f, %f]', msg.x, msg.y)
        )

        rospy.loginfo("init done")

    def run(self):
        while self.keep_alive():
            self.spin()
            self.rate.sleep()

    def keep_alive(self):
        return not rospy.is_shutdown()

    def spin(self):
        rospy.loginfo('[%f, %f, %f]', self.state.x, self.state.y, self.state.yaw)
        self.rviz.log_state(self.state)
        self.rviz.visualize_data()


if __name__ == '__main__':

    ## Start node ##

    state_publisher().run()

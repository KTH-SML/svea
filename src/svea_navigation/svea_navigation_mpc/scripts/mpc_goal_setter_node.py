#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from svea_navigation_mpc.srv import SetGoalPosition, SetGoalPositionRequest

class GoalSetterNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('goal_setter_node')

        # Subscribe to the clicked_point topic to receive goal position from RViz
        rospy.Subscriber('/mpc_target', PoseStamped, self.clicked_point_callback)

        # Service client to call the set_goal_position service
        rospy.wait_for_service('/set_goal_position')
        self.set_goal_service = rospy.ServiceProxy('/set_goal_position', SetGoalPosition)

        self.goal_pose = None

    def clicked_point_callback(self, msg):
        """
        Callback to receive the goal position when the user clicks a point in RViz.
        """
        # Convert PointStamped to PoseStamped
        self.goal_pose = PoseStamped()
        self.goal_pose = msg

        rospy.loginfo(f"Received clicked point: ({self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y})")

        # Call the service to send the goal to the main node
        self.send_goal_to_main_node()

    def send_goal_to_main_node(self):
        """
        Calls the service offered by the MainNode, sending the goal position.
        """
        if self.goal_pose is None:
            rospy.logwarn("No goal position set yet.")
            return

        # Create a request object for the service
        req = SetGoalPositionRequest()
        req.goal_pose = self.goal_pose

        # Call the service and pass the goal position
        try:
            response = self.set_goal_service(req)
            if response.success:
                rospy.loginfo(f"Service call successful: {response.message}")
            else:
                rospy.logwarn(f"Service call failed: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = GoalSetterNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

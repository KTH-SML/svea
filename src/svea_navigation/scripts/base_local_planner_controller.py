import rospy
from geometry_msgs.msg import Twist
import math

class BaseLocalPlannerController(object):

    def __init__(self, vehicle_name=''):
        rospy.Subscriber("/cmd_vel", Twist, self.blp_control, queue_size=1)
        self.vehicle_name = vehicle_name
        self.steering = 0.0
        self.velocity = 0.0
        self.wheelbase = 0.324  # Distance between the front and rear axles (in meters)

    def blp_control(self, data):
        linear_velocity = data.linear.x
        angular_velocity = data.angular.z

        # Convert angular velocity to steering angle
        if angular_velocity != 0:
            self.steering = math.atan2(self.wheelbase * angular_velocity, linear_velocity)
        else:
            self.steering = 0.0

        self.velocity = linear_velocity


    def compute_control(self, state):
        return self.steering, self.velocity

if __name__ == "__main__":
    rospy.init_node('base_local_planner_controller')
    controller = BaseLocalPlannerController()
    rospy.spin()

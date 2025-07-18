#! /usr/bin/env python3

from tf_transformations import quaternion_from_euler

from svea_core.interfaces import LocalizationInterface
from svea_core.interfaces import ActuationInterface
from svea_core import rosonic as rx
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

qos_subber = QoSProfile(depth=10 # Size of the queue 
                        )

class teleop_control(rx.Node):  # Inherit from rx.Node

    DELTA_TIME = 0.1

    is_sim = rx.Parameter(True)
    target_velocity = rx.Parameter(1.0)

    actuation = ActuationInterface()
    localizer = LocalizationInterface()

    ## Subscribers ##
    @rx.Subscriber(Twist, 'cmd_vel', qos_subber)
    def ctrl_request_twist(self, twist_msg):
        self.velocity = twist_msg.linear.x
        self.steering = -twist_msg.angular.z

    def on_startup(self):

        self.velocity = 0.0
        self.steering = 0.0

        state = self.localizer.get_state()

        self.create_timer(self.DELTA_TIME, self.loop)

    def loop(self):
        state = self.localizer.get_state()
        
        self.actuation.send_control(self.steering, self.velocity)



if __name__ == '__main__':
    teleop_control.main()
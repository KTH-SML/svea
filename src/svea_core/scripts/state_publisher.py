#! /usr/bin/env python3

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

from svea_core import rosonic as rx
from svea_core import LocalizationInterface


class state_publisher(rx.Node):
    """
    A quick and easy way to get vehicle states.

    Launch the corresponding launch file and this node will continuously print
    out the vehicle state (coordinates). Useful when finding coordinates in a
    map.
    """

    rate = rx.Parameter(10)

    def on_startup(self):

        self.odometry = Odometry()
        
        self.rate = self.create_rate(self.rate)

        self.localization = LocalizationInterface(self).start()

        self.create_subscription(
            PointStamped,
            '/clicked_point',
            lambda msg: self.get_logger().info('Clicked point: [%f, %f]' % (msg.point.x, msg.point.y)),
            10
        )

        self.create_timer(1/self.rate, self.loop)

    def loop(self):
        """Main loop for the state publisher."""
        
        while rclpy.ok():

            state = self.localization.get_state()
            self.get_logger().info(
                "(" + ", ".join(f'{val:.02f}' for val in state) + ")"
            )

            self.rate.sleep()


if __name__ == '__main__':
    StatePublisher.main()

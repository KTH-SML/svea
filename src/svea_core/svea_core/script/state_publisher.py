#! /usr/bin/env python3

import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

from svea_core.interfaces import LocalizationInterface


class StatePublisher(Node):
    """
    A quick and easy way to get vehicle states.

    Launch the corresponding launch file and this node will continuously print
    out the vehicle state (coordinates). Useful when finding coordinates in a
    map.
    """

    RATE = 10

    def __init__(self):
        super().__init__('state_publisher')

        ## Create node resources
        
        self.odometry = Odometry()
        
        self.rate = self.create_rate(self.RATE)

        self.loc = LocalizationInterface(self).start()

        # Clicked point listener
        self.sub_clicked_point = self.create_subscription(
            PointStamped,
            '/clicked_point',
            lambda msg: self.get_logger().info('Clicked point: [%f, %f]' % (msg.point.x, msg.point.y)),
            10
        )

        self.get_logger().info("init done")

    def run(self):
        while self.keep_alive():
            self.rate.sleep()
            self.loop()

    def keep_alive(self):
        return rclpy.ok()

    def spin(self):
        state = self.loc.get_state()
        self.get_logger().info(
            "(" + ", ".join(f'{val:.02f}' for val in state) + ")"
        )


def main(args=None):
    rclpy.init(args=args)
    StatePublisher().run()


if __name__ == '__main__':
    main()

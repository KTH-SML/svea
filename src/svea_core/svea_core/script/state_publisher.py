#! /usr/bin/env python3

import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion



class StatePublisher(Node):
    """
    A quick and easy way to get vehicle states.

    Launch the corresponding launch file and this node will continuously print
    out the vehicle state (coordinates). Useful when finding coordinates in a
    map.
    """

    SPIN_RATE = 10

    def __init__(self):
        super().__init__('state_publisher')

        ## Create node resources
        self.odometry = Odometry()
        # spin rate
        self.rate = self.create_rate(self.SPIN_RATE)
        
        # Wait for the first state message
        self.state.state_msg = self.wait_for_message('odometry/local', Odometry)

        self.create_subscription(
            Odometry,
            'odometry/local',
            lambda msg: setattr(self.odometry, 'odometry/local', msg),
            10
        )

        # Clicked point listener
        self.sub_clicked_point = self.create_subscription(
            PointStamped,
            '/clicked_point',
            lambda msg: self.get_logger().info('Clicked point: [%f, %f]' % (msg.point.x, msg.point.y)),
            10
        )

        self.get_logger().info("init done")

    def wait_for_message(self, topic, msg_type):
        future = rclpy.task.Future()
        sub = self.create_subscription(
            msg_type,
            topic,
            lambda msg: future.set_result(msg),
            10
        )
        rclpy.spin_until_future_complete(self, future)
        self.destroy_subscription(sub)
        return future.result()

    def run(self):
        while self.keep_alive():
            self.spin()
            self.rate.sleep()

    def keep_alive(self):
        return rclpy.ok()

    def spin(self):
        quaternion = [self.odometry.pose.pose.orientation.x,
                       self.odometry.pose.pose.orientation.y,
                       self.odometry.pose.pose.orientation.z,
                       self.odometry.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.get_logger().info('[%f, %f, %f]' % (self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y, yaw))


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    node.run()


if __name__ == '__main__':
    main()




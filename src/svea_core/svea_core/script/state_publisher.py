#! /usr/bin/env python3

import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import PointStamped
from svea_msgs.msg import VehicleState as VehicleStateMsg
from svea_core.states import VehicleState



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

        # spin rate
        self.rate = self.create_rate(self.SPIN_RATE)

        # Vehicle state
        self.state = VehicleState()
        
        # Wait for the first state message
        # self.state.state_msg = self.wait_for_message('state', VehicleStateMsg)
        self.create_subscription(
            VehicleStateMsg,
            'state',
            lambda msg: setattr(self.state, 'state_msg', msg),
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
        self.get_logger().info('[%f, %f, %f]' % (self.state.x, self.state.y, self.state.yaw))


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    node.run()


if __name__ == '__main__':
    main()




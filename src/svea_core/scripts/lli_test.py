#!/usr/bin/env python3

"""
Node to test the lli control interface by giving different directional commands to the car.
"""

import rclpy
import rclpy.clock
from rclpy.node import Node
import rclpy.logging
import math
from svea_core.interfaces import ActuationInterface
# import zenoh 

class LLITesting(Node):

    rate = 10
    run_time = 3
    test_speed = 0.3 # [m/s]
    test_angle = math.pi/8 # [rad] approx 22.5 degrees

    def __init__(self):
        super().__init__('LLITesting')
        self.ctrl_interface = ActuationInterface(self).start(wait = True)
        self.r = self.create_rate(self.rate)
        self.run_time_duration = rclpy.clock.Duration(seconds=self.run_time).nanoseconds

    def foward(self):
        self.get_logger().info("Going forward for 3 seconds")
        i = 0
        while rclpy.ok() and i < self.rate*self.run_time:
            self.ctrl_interface.send_control(
                steering=0,
                velocity=self.test_speed,
                transmission=0)
            self.get_logger().info(self.ctrl_interface, throttle_duration_sec=self.run_time_duration)
            self.r.sleep()
            i += 1

    def backward(self):
        self.get_logger().info("Going backwards for 3 seconds")
        i = 0
        while rclpy.ok() and i < self.rate*self.run_time:
            self.ctrl_interface.send_control(0, -self.test_speed)
            self.get_logger().info(self.ctrl_interface, throttle_duration_sec=self.run_time_duration)
            self.r.sleep()
            i += 1

    def brake(self):
        self.get_logger().info("Braking for 3 seconds")
        i = 0
        while rclpy.ok() and i < self.rate*self.run_time:
            self.ctrl_interface.send_control(brake_force=20)
            self.get_logger().info(self.ctrl_interface, throttle_duration_sec=self.run_time_duration)
            self.r.sleep()
            i += 1
    
    def turn_left(self):
        self.get_logger().info("Turning left for 3 seconds")
        i = 0
        while rclpy.ok() and i < self.rate*self.run_time:
            self.ctrl_interface.send_control(self.test_angle, 0.0)
            self.get_logger().info(self.ctrl_interface, throttle_duration_sec=self.run_time_duration)
            self.r.sleep()
            i += 1
    
    def turn_right(self):
        self.get_logger().info("Turning right for 3 seconds")
        i = 0
        while rclpy.ok() and i < self.rate*self.run_time:
            self.ctrl_interface.send_control(-self.test_angle, 0.0)
            self.get_logger().info(self.ctrl_interface, throttle_duration_sec=self.run_time_duration)
            self.r.sleep()
            i += 1

    def run(self):
        print('running something')



def main(args=None):
    """
    Starts instance of ControlInterface and waits for initialization.
    Sends move forward, move backward, turn left and turn right commands to the car for 3 seconds each.
    """
    
    rclpy.init(args=args)
    testing = LLITesting()
    testing.run()
    testing.foward()
    testing.backward()
    testing.brake()
    testing.backward()
    testing.turn_left()
    testing.turn_right()

    rclpy.logging.get_logger('lli testing').info("Going idle")
    rclpy.spin(testing)
    rclpy.shutdown()

if __name__ == '__main__': 
    main()

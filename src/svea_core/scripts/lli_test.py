#!/usr/bin/env python3

import rclpy

from svea_core import rosonic as rx
from svea_core.interfaces import ActuationInterface

class LLITest(rx.Node):
    """Test LLI interface."""

    RATE = 10 # [Hz]
    RUNTIME = 3 # [s]
    SPEED = 0.3 # [m/s]
    ANGLE = 0.3 # [rad]

    actuation = ActuationInterface()

    def on_startup(self):
        self.rate = self.create_rate(self.RATE)

    def forward(self):
        self.get_logger().info("Going forward for 3 seconds")
        for _ in range(self.RUNTIME * self.RATE):
            if rclpy.ok():
                self.actuation.send_control(steering=0, velocity=self.SPEED)
                self.rate.sleep()
        
    def backward(self):
        self.get_logger().info("Going backwards for 3 seconds")
        for _ in range(self.RUNTIME * self.RATE):
            if rclpy.ok():
                self.actuation.send_control(steering=0, velocity=-self.SPEED)
                self.rate.sleep()

    def turn_left(self):
        self.get_logger().info("Turning left for 3 seconds")
        for _ in range(self.RUNTIME * self.RATE):
            if rclpy.ok():
                self.actuation.send_control(steering=self.ANGLE, velocity=0)
                self.rate.sleep()      

    def turn_right(self):
        self.get_logger().info("Turning right for 3 seconds")
        for _ in range(self.RUNTIME * self.RATE):
            if rclpy.ok():
                self.actuation.send_control(steering=-self.ANGLE, velocity=0)
                self.rate.sleep()

    def run(self):
        self.forward()
        self.backward()
        self.turn_left()
        self.turn_right()


if __name__ == '__main__': 
    LLITest.main()

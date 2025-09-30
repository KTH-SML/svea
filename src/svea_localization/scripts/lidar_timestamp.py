#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class TimestampCorrector(Node):
    def __init__(self):
        super().__init__('timestamp_corrector')
        self.subscription = self.create_subscription(
            LaserScan, '/scan_raw', self.scan_callback, 10)
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
    
    def scan_callback(self, msg):
        # 使用当前时间替换时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

def main():
    rclpy.init()
    corrector = TimestampCorrector()
    rclpy.spin(corrector)
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid


class PlotMap(Node):

    map = None

    def __init__(self):

        super().__init__('plot_map')

        
        
        self.sub_map = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_cb,
            10
        )

    def spin(self):
        if not self.map is None:
            plt.imshow(self.map[:, :])
            plt.show()

    def map_cb(self, msg: OccupancyGrid):
        height = msg.info.height
        width = msg.info.width
        data = msg.data
        self.map = np.array(data).reshape(height, width)

def main(args=None):
    rclpy.init(args=args)
    plot_map = PlotMap()
    rate = plot_map.create_rate(2)
    try:
        while rclpy.ok():
            rclpy.spin_once(plot_map, timeout_sec=0.1)
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        plot_map.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

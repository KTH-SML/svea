#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid


class plot_map:

    map = None

    def __init__(self):

        rospy.init_node('plot_map')

        self.rate = rospy.Rate(2)

        self.map_cb(rospy.wait_for_message("/map", OccupancyGrid))
        self.sub_map = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)

    def run(self):
        while not rospy.is_shutdown():
            self.spin()
            self.rate.sleep()

    def spin(self):
        if not self.map is None:
            plt.imshow(self.map[:, :])
            plt.show()

    def map_cb(self, msg: OccupancyGrid):
        height = msg.info.height
        width = msg.info.width
        data = msg.data
        self.map = np.array(data).reshape(height, width)


if __name__ == '__main__':
    plot_map().run()

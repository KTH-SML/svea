#!/usr/bin/env python

import sys
import os
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt

dirname = os.path.dirname(__file__)
svea = os.path.join(dirname, '../../')
sys.path.append(os.path.abspath(svea))

from nav_msgs.msg import OccupancyGrid

svea_map = None
map_meta = None

def update_map(map_msg):
    global svea_map, map_meta
    map_meta = map_msg.info
    map_data = map_msg.data

    height = map_meta.height
    width = map_meta.width
    svea_map = np.array(map_data).reshape(height, width)
    svea_map = np.flip(svea_map, 0)

def main():
    rospy.init_node('map_plt')

    rospy.Subscriber("/map", OccupancyGrid, update_map)

    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        if not svea_map is None:
            imgplot = plt.imshow(svea_map[:, :])
            plt.show()
        r.sleep()


if __name__ == '__main__':
    main()

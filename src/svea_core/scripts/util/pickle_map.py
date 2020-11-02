#!/usr/bin/env python

import os
import pickle
import rospy
from nav_msgs.msg import OccupancyGrid

dirname = os.path.dirname(__file__)
svea_core = os.path.join(dirname, '../../')
map_name = "floor2" # change this for different maps
file_path = svea_core + 'resources/maps/' + map_name + ".pickle"

saved = False
def save_map(map_msg):
    global saved
    f = open(file_path, "wb")
    pickle.dump(map_msg, f)
    saved = True

def main():
    rospy.init_node('map_plt')
    rospy.Subscriber("/map", OccupancyGrid, save_map)
    while not rospy.is_shutdown() and not saved:
        pass

if __name__ == '__main__':
    main()

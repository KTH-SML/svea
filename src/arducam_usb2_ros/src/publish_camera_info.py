#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo
import yaml

class publish_camera_info():
    def __init__(self):
        self.pub = rospy.Publisher("/camera/arducam/camera/camera_info", CameraInfo, queue_size=1)
        self.seq = 0
        # self.config = yaml.load(open(rospy.get_param("~config_file", "")))
        # print(self.config)
    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def publish_msg_camera_info(self, img_header):
        if not rospy.is_shutdown():
            msg = CameraInfo()
            msg.header.seq = img_header.seq
            msg.header.stamp = img_header.stamp
            msg.header.frame_id = img_header.frame_id
            msg.height = 480
            msg.width = 640
            msg.distortion_model = "plumb_bob"
            msg.D = [-0.373435, 0.163346, 0.000573, 0.000605, 0.000000]
            msg.K = [982.668058, 0.000000, 294.004189, 0.000000, 985.022219, 277.250847, 0.000000, 0.000000, 1.000000]
            msg.R = [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
            msg.P = [941.726616, 0.000000, 292.009434, 0.000000, 0.000000, 960.730864, 279.083164, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
            # msg.binning_x
            # msg.binning_y
            # msg.roi
            self.seq += 1
            self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('publish_camera_info')
    pub_obj = publish_camera_info()
    pub_obj.run()
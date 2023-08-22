#! /usr/bin/env python3

import numpy as np

import rospy

# import tf2_ros
# import tf2_geometry_msgs
# from tf import transformations 
# from tf.transformations import euler_from_quaternion, quaternion_from_euler

# from sensor_msgs.msg import NavSatFix
# from aruco_msgs.msg import Marker


import tf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovariance, Pose, Quaternion, Point
from rsu_msgs.msg import Object, ObjectPose, StampedObjectArray, StampedObjectPoseArray

####################################################
###Dummy publisher to test pedestrian_location.py###
###Publish 3 pedestrians to sensor/objects topic####
####################################################

class publish_rsu:

    def __init__(self):
        #initalize the node 
        

        #Publisher
        self.test_Pub = rospy.Publisher("sensor/objects", StampedObjectPoseArray, queue_size=1) 
        
        self.ped_pose = [[0, 0, 0], [1, 2, 0], [3, 3, 0]]
        self.ped_ori  = [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]]
        self.seq_count = 0

    def pub(self): #object @ UTM
        msg = StampedObjectPoseArray()
        msg.header.seq = self.seq_count
        self.seq_count += 1
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "RSU"
        for i in range(3):
            obj = ObjectPose()
            obj.object.id = i
            obj.object.label = "peds"
            obj.object.detection_conf = np.random.rand()
            obj.object.tracking_conf = np.random.rand()
            obj.object.image_width = 640
            obj.object.image_height = 480
            # obj.object.RegionOfInterest = 

            obj.pose.pose.position = Point(*self.ped_pose[i])
            obj.pose.pose.orientation = Quaternion(*self.ped_ori[i])
            msg.objects.append(obj)
        self.test_Pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('publish_rsu')

    r = rospy.Rate(10) 
    while not rospy.is_shutdown():
        publish_rsu().pub()
        r.sleep()
    
#! /usr/bin/env python3

import numpy as np

import rospy

import tf2_ros
import tf2_geometry_msgs
from tf import transformations 
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from sensor_msgs.msg import NavSatFix
from aruco_msgs.msg import Marker


import tf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovariance, Pose, TransformStamped
from rsu_msgs.msg import Object, ObjectPose, StampedObjectArray, StampedObjectPoseArray

class pedestrian_location:

    def __init__(self):
        #initalize the node 
        rospy.init_node('pedestrian_location')

        self.pedestrian_pose = rospy.get_param("~pedestrian_pose", "sensor/objects")

        #Subscriber
        rospy.Subscriber(self.pedestrian_pose, StampedObjectPoseArray, self.pedestrian_callback, queue_size=1)
        
        #Publisher
        self.ped_pose_in_SVEA_Pub = rospy.Publisher("pedestrianPoseInSVEA", StampedObjectPoseArray, queue_size=1) 

        #Transformation
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.pedestrian_utm = []
        self.pedestrian_obj = []

        self.pedestrian_SVEA = []

        self.rsu_utm_transform = None

    def run(self):
        rospy.spin()

    def pedestrian_callback(self,msg):
        while True and not rospy.is_shutdown():
            try:
                self.rsu_utm_transform = self.buffer.lookup_transform("utm", "RSU", rospy.Time.now(), rospy.Duration(0.5))
                # rospy.loginfo(f'rsu utm \n {self.rsu_utm_transform}')
                break
            except:
                rospy.loginfo(f"Transforming RSU @ UTM frame")
        self.pedestrian_utm = []
        self.pedestrian_obj = []
        for obj in msg.objects: #object in RSU frame
            obj_in_utm = tf2_geometry_msgs.do_transform_pose(obj.pose, self.rsu_utm_transform) #Object in UTM frame
            self.pedestrian_obj.append(obj.object)
            self.pedestrian_utm.append(obj_in_utm)
        self.publish_tf(self.pedestrian_utm, self.pedestrian_obj)
        self.pedestrian_in_svea(msg.header, self.pedestrian_utm, self.pedestrian_obj)

    def pedestrian_in_svea(self, header, pedestrian_utm, pedestrian_obj):
        self.pedestrian_SVEA = []
        try:
            SVEA_in_utm = self.buffer.lookup_transform("base_link", "utm", header.stamp, rospy.Duration(0.5)) #UTM @ SVEA
            # SVEA_pose = Pose()
            # SVEA_pose.position = SVEA_in_utm.transform.translation
            # SVEA_pose.orientation = SVEA_in_utm.transform.rotation
    
            for pedestrian in pedestrian_utm:
                object_in_SVEA = tf2_geometry_msgs.do_transform_pose(pedestrian, SVEA_in_utm) #object @ SVEA
                self.pedestrian_SVEA.append(object_in_SVEA)
    
            self.publish_topic(self.pedestrian_SVEA, pedestrian_obj)
        except Exception as e:
            rospy.loginfo(e)

    def publish_topic(self, pedestrian_SVEA, pedestrian_obj):
        msg = StampedObjectPoseArray()
        msg.header = pedestrian_SVEA[0].header
        for peds, info in zip(pedestrian_SVEA, pedestrian_obj):
            obj = ObjectPose()
            obj.object = info
            obj.pose.pose.position = peds.pose.position
            obj.pose.pose.orientation = peds.pose.orientation
            msg.objects.append(obj)
        self.ped_pose_in_SVEA_Pub.publish(msg)

        for peds, info in zip(pedestrian_SVEA, pedestrian_obj):
            msg = TransformStamped()
            msg.header = peds.header
            msg.child_frame_id = info.label + str(info.id) + "SVEA"
            msg.transform.translation = peds.pose.position
            msg.transform.rotation = peds.pose.orientation
            self.br.sendTransform(msg)

    def publish_tf(self, pedestrian_utm, pedestrian_obj): #object @ UTM
        for peds, info in zip(pedestrian_utm, pedestrian_obj):
            msg = TransformStamped()
            msg.header = peds.header
            msg.child_frame_id = info.label + str(info.id)
            msg.transform.translation = peds.pose.position
            msg.transform.rotation = peds.pose.orientation
            self.br.sendTransform(msg)

if __name__ == '__main__':
    pedestrian_location().run()
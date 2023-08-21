#! /usr/bin/env python3

import numpy as np

import rospy

import tf
import tf2_ros
import tf2_geometry_msgs
from tf import transformations 
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Point, Quaternion, PoseStamped
from aruco_msgs.msg import Marker

class aruco_pose:

    def __init__(self):
        #initalize the node 
        rospy.init_node('aruco_pose')

        self.gps_fix_topic = rospy.get_param("~static_gps_fix_topic", "static/gps/fix")
        self.odometry_gps_topic = rospy.get_param("~odometry_gps_topic", "static/odometry/gps")
        self.aruco_pose_topic = rospy.get_param("~aruco_pose_topic", "aruco_pose")

        self.aruco_id = rospy.get_param("~aruco_id", 11)

        #Subscriber
        rospy.Subscriber(self.aruco_pose_topic, Marker, self.aruco_callback, queue_size=1)
        
        #Publisher
        self.pose_pub = rospy.Publisher("static/pose", PoseWithCovarianceStamped, queue_size=1) #publish to pose0 in ekf
        
        #Variable
        self.gps_msg = None
        self.location = None

        self.frame = 'aruco' + str(self.aruco_id)

        #Transformation
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.lin_cov = 1e-6
        self.ang_cov = 1e-6

    def run(self):
        rospy.spin()

    def aruco_callback(self, msg):    
        if msg.id == self.aruco_id:
            if (msg.pose.pose.position.x**2 + msg.pose.pose.position.y**2 + msg.pose.pose.position.z**2) <= 1.5:
                try:
                    
                    transform_aruco_map = self.buffer.lookup_transform("map", 'aruco11_actual', rospy.Time.now(), rospy.Duration(0.5)) #frame_id: map, child_frame_id: aruco
                    transform_baselink_aruco = self.buffer.lookup_transform(self.frame, "base_link", msg.header.stamp, rospy.Duration(0.5)) #frame_id: aruco , child_frame_id: base_link

                    pose_aruco_baselink = PoseStamped()
                    pose_aruco_baselink.header = transform_baselink_aruco.header
                    pose_aruco_baselink.pose.position = transform_baselink_aruco.transform.translation
                    pose_aruco_baselink.pose.orientation = transform_baselink_aruco.transform.rotation

                    adjust_orientation = TransformStamped()
                    adjust_orientation.header = msg.header
                    adjust_orientation.header.frame_id = "map"
                    adjust_orientation.child_frame_id = 'aruco11'
                    adjust_orientation.transform.rotation = Quaternion(*quaternion_from_euler(1.57, 0, 1.57))

                    position = tf2_geometry_msgs.do_transform_pose(pose_aruco_baselink, adjust_orientation) #frame_id = map child_frame: baselink

                    position_final = tf2_geometry_msgs.do_transform_pose(position, transform_aruco_map) #frame_id = map child_frame: baselink
                    position_final.pose.position.z = 0.0

                    self.publish_pose(position_final.pose.position, position_final.pose.orientation, msg.header.stamp)
                    self.broadcast_aruco(position_final.pose.position, position_final.pose.orientation, msg.header.stamp)

                    rospy.loginfo("Received ARUCO")
                except Exception as e:
                    rospy.logerr(e)

    def publish_pose(self, translation, quaternion, time):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = time
        msg.header.frame_id = "map"
        msg.pose.pose.position = translation
        msg.pose.pose.orientation = quaternion
        self.cov_matrix = [self.lin_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, self.lin_cov, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, self.lin_cov, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, self.ang_cov, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, self.ang_cov, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, self.ang_cov]
        msg.pose.covariance = self.cov_matrix
        self.pose_pub.publish(msg)

    def broadcast_aruco(self, translation, quaternion, time):
        msg = TransformStamped()
        msg.header.stamp = time
        msg.header.frame_id = "map"
        msg.child_frame_id = "basebase"
        msg.transform.translation = translation
        msg.transform.rotation = quaternion
        self.br.sendTransform(msg)

if __name__ == '__main__':
    aruco_pose().run()
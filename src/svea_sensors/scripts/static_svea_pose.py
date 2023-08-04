#! /usr/bin/env python3

import numpy as np

import rospy

import tf
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Point, Quaternion, PoseStamped
from aruco_msgs.msg import Marker

#TODO: add the transform from svea to aruco marker 
#TODO: publish a statiic/pose for EKF global 

#TODO: conditions for static gps and active gps

class static_svea_gps:

    def __init__(self):
        #initalize the node 
        rospy.init_node('static_svea_gps')

        self.gps_fix_topic = rospy.get_param("~static_gps_fix_topic", "static/gps/fix")
        self.odometry_gps_topic = rospy.get_param("~odometry_gps_topic", "static/odometry/gps")
        self.aruco_pose_topic = rospy.get_param("~aruco_pose_topic", "aruco_pose")

        self.aruco_id = rospy.get_param("~aruco_id", 11)

        #Subscriber
        rospy.Subscriber(self.gps_fix_topic, NavSatFix, self.gps_callback) # gps signal (lat long) of static svea
        rospy.Subscriber(self.odometry_gps_topic, TFMessage, self.odom_gps_callback) # position of static svea
        rospy.Subscriber(self.aruco_pose_topic, Marker, self.aruco_callback)
        
        #Publisher
        self.pose_pub = rospy.Publisher("static/pose", PoseWithCovarianceStamped, queue_size=10) #publish to pose0 in ekf
        
        #Variable
        self.gps_msg = None
        self.location = None

        self.frame = 'aruco' + str(self.aruco_id)

        #Transformation
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.br = tf2_ros.TransformBroadcaster()

    def run(self):
        rospy.spin()

    def gps_callback(self, msg):
        if self.gps_msg is None and np.sum(msg.position_covariance) <= 1e-4:
            self.gps_msg = msg

    def odom_gps_callback(self, msg):
        if self.location is None:           
            self.location = PoseWithCovarianceStamped()
            self.location.header = msg.header
            self.location.pose = msg.pose

    def aruco_callback(self, msg):
        if msg.id == self.aruco_id:
            try:
                transform = self.buffer.lookup_transform('base_link', self.frame, self.aruco_header, rospy.Duration(0.5))

                self.broadcast_aruco(transform.transform.translation, transform.transform.rotation, self.aruco_header)
                self.publish_pose(transform.transform.translation, transform.transform.rotation, self.aruco_header)

            except Exception as e: 
                rospy.loginfo(f"Error, {e}")

    def publish_pose(self, translation, quaternion, time):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = time
        msg.header.frame_id = "map"
        msg.pose.pose.position = translation
        msg.pose.pose.orientation = quaternion
        self.pose_pub.publish(msg)

    def broadcast_aruco(self, translation, quaternion, time):
        msg = TransformStamped()
        msg.header.stamp = time
        msg.header.frame_id = "base_link"
        msg.child_frame_id = "arucofromtransform"
        msg.transform.translation = translation
        msg.transform.rotation = quaternion
        self.br.sendTransform(msg)

if __name__ == '__main__':
    static_svea_gps().run()
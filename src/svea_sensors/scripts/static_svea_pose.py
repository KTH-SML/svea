#! /usr/bin/env python3

import numpy as np

import rospy

import tf
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Point, Quaternion
from aruco_msgs.msg import Marker

#TODO: add the transform from svea to aruco marker 
#TODO: publish a statiic/pose for EKF global 

#TODO: conditions for static gps and active gps

class static_svea_gps:

    def __init__(self):
        #initalize the node 
        rospy.init_node('static_svea_gps')

        self.gps_fix_topic = rospy.get_param("~gps_fix", "static/gps/fix")
        self.odometry_gps_topic = rospy.get_param("~odometry_gps", "static/odometry/gps")
        self.aruco_pose_topic = rospy.get_param("~aruco_pose", "aruco_pose")

        #Subscriber
        rospy.Subscriber(self.gps_fix_topic, NavSatFix, self.gps_callback) # gps signal (lat long) of static svea
        rospy.Subscriber(self.odometry_gps_topic, TFMessage, self.odom_gps_callback) # position of static svea
        rospy.Subscriber(self.aruco_pose_topic, Marker, self.aruco_callback)
        
        #Publisher
        self.pose_pub = rospy.Publisher("static/pose", PoseWithCovarianceStamped, queue_size=10) #publish to pose0 in ekf
        
        #Variable
        self.gps_msg = None
        self.location = None

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
        self.publish_pose()

    def aruco_callback(self, msg):
        try:
            self.aruco_header = msg.header.stamp
            transform = self.buffer.lookup_transform('camera', 'aruco11', self.aruco_header, rospy.Duration(0.5))
            self.broadcast_aruco(transform.transform.translation, transform.transform.rotation, self.aruco_header)
        except Exception as e: 
            rospy.loginfo(f"Error, {e}")

    def publish_pose(self):
        self.pose_pub.publish(self.location)

    def broadcast_aruco(self, translation, quaternion, time):
        msg = TransformStamped()
        msg.header.stamp = time
        msg.header.frame_id = "camera"
        msg.child_frame_id = "arucofromtransform"
        msg.transform.translation = translation
        msg.transform.rotation = quaternion
        self.br.sendTransform(msg)

if __name__ == '__main__':
    static_svea_gps().run()
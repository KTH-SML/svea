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

        self.origin_x_aruco = 0
        self.origin_y_aruco = 0
        
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
        if msg.id == 11:
#            try:
                #CORRECT ARUCO11 TO MAP
                self.aruco_header = msg.header.stamp
                self.aruco_pose = PoseStamped()
                self.aruco_pose.header.stamp = rospy.Time.now() #from map to aruco11
                self.aruco_pose.header.frame_id = "map"
                self.aruco_pose.pose.position.x = 0.0
                self.aruco_pose.pose.position.y = 0.0
                self.aruco_pose.pose.orientation.w = 1.0

#                transform = self.buffer.lookup_transform('aruco11', 'camera', self.aruco_header, rospy.Duration(0.5))
                
                #MEASURED ARUCO11 TO MAP
                transform = self.buffer.lookup_transform('map', 'aruco11', self.aruco_header, rospy.Duration(0.5))
                rospy.loginfo(f"transform{transform}")
                
                #ERROR BETWEEN CORRECT AND MEASURED
                error = Point()
                error.x = self.aruco_pose.pose.position.x - transform.transform.translation.x
                error.y = self.aruco_pose.pose.position.y - transform.transform.translation.y
                error.z = self.aruco_pose.pose.position.z - transform.transform.translation.z

                transform2 = self.buffer.lookup_transform('map', 'base_link', self.aruco_header, rospy.Duration(0.5))
                final_transform = self.buffer.lookup_transform('base_link', 'camera', self.aruco_header, rospy.Duration(0.5))
#                final_transform.transform.translation.x = transform2.transform.translation.x - error.x
#                final_transform.transform.translation.y = transform2.transform.translation.y - error.y
#                final_transform.transform.translation.z = transform2.transform.translation.z - error.z
                rospy.loginfo(f"final_transform{final_transform}")
                rospy.loginfo(f"error{error}")

                self.broadcast_aruco(final_transform.transform.translation, final_transform.transform.rotation, self.aruco_header)
                self.publish_pose(final_transform.transform.translation, final_transform.transform.rotation, self.aruco_header)

#attempt2
#                transform = self.buffer.lookup_transform('camera', 'aruco11', self.aruco_header, rospy.Duration(0.5))
#                transform2 = self.buffer.lookup_transform('map', 'camera', self.aruco_header, rospy.Duration(0.5))
#                rospy.loginfo(f"transform{transform2}")

#attempt1
#                transform = self.buffer.lookup_transform('camera', 'aruco11', self.aruco_header, rospy.Duration(0.5))
#                do_trans = tf2_geometry_msgs.do_transform_pose(self.aruco_pose, transform)
#                self.broadcast_aruco(do_trans.pose.position, do_trans.pose.orientation, self.aruco_header)
#                self.publish_pose(do_trans.pose.position, do_trans.pose.orientation, self.aruco_header)

#            except Exception as e: 
#                rospy.loginfo(f"Error, {e}")

    def publish_pose(self, translation, quaternion, time):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = time
        msg.header.frame_id = "camera"
        msg.pose.pose.position = translation
        msg.pose.pose.orientation = quaternion
        self.pose_pub.publish(msg)

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
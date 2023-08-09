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

        self.lin_cov = 0.1
        self.ang_cov = 0.1

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
#            rospy.loginfo("Received ARUCO")

        ####################### INVERSE TRANSFORM, SAME AS LOOKUP FROM ARUCO TO BASELINK ##############################
            '''transform = self.buffer.lookup_transform("base_link", "camera", msg.header.stamp, rospy.Duration(0.5)) #frame_id: baselink, child_frame_id: camera
            
            position = tf2_geometry_msgs.do_transform_pose(msg.pose, transform) #frame_id = baselink child_frame: aruco
            translation_mat = transformations.translation_matrix([position.pose.position.x,
                                                        position.pose.position.y,
                                                        position.pose.position.z])
            rotation_mat = transformations.quaternion_matrix([position.pose.orientation.x,
                                                position.pose.orientation.y,
                                                position.pose.orientation.z,
                                                position.pose.orientation.w])
            transform_mat = transformations.concatenate_matrices(translation_mat, rotation_mat)
            inversed_trans = transformations.inverse_matrix(transform_mat)

            inv_transla = Point(*transformations.translation_from_matrix(inversed_trans)) #frame_id = aruco child_frame: baselink
            inv_rotate = Quaternion(*transformations.quaternion_from_matrix(inversed_trans))
            rospy.loginfo(f"inv \n {inv_transla} \n {inv_rotate}")'''
            
        ##

            transform = self.buffer.lookup_transform(self.frame, "base_link", msg.header.stamp, rospy.Duration(0.5)) #frame_id: aruco, child_frame_id: baselink

            transform2 = self.buffer.lookup_transform("odom", self.frame, msg.header.stamp, rospy.Duration(0.5)) #frame_id: odom , child_frame_id: aruco
            transform2.transform.translation = Point(*[0, 0, 0])
            
            temp = PoseStamped()
            temp.pose.position = transform.transform.translation
            temp.pose.orientation.w = 1.0   

            position = tf2_geometry_msgs.do_transform_pose(temp, transform2) #frame_id = odom child_frame: baselink
            position.pose.position.z = 0.0
            self.publish_pose(position.pose.position, position.pose.orientation, msg.header.stamp)
#            self.broadcast_aruco(position.pose.position, position.pose.orientation, msg.header.stamp)


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
    static_svea_gps().run()
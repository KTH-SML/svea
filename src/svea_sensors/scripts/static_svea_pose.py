#! /usr/bin/env python3

import numpy as np

import rospy

import tf
import tf2_ros
import tf2_geometry_msgs
from tf import transformations 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
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
        rospy.Subscriber('/odometry/filtered/global', Odometry, self.current_pose_callback)
        
        #Publisher
        self.pose_pub = rospy.Publisher("static/pose", PoseWithCovarianceStamped, queue_size=10) #publish to pose0 in ekf
        self.global_set_pose_pub = rospy.Publisher("/global/set_pose", PoseWithCovarianceStamped, queue_size=1) #publish to set_pose service provided by robot localization to reset the position
        self.set_pose_pub = rospy.Publisher("/set_pose", PoseWithCovarianceStamped, queue_size=1) #publish to set_pose service provided by robot localization to reset the position

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

        self.current_ori = None

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

            transform = self.buffer.lookup_transform(self.frame, "base_link", msg.header.stamp, rospy.Duration(0.5)) #frame_id: aruco, child_frame_id: baselink
            
            transform2 = self.buffer.lookup_transform("odom", self.frame, msg.header.stamp, rospy.Duration(0.5)) #frame_id: odom , child_frame_id: aruco
            transform2.transform.translation = Point(*[0, 0, 0])
            
            temp = PoseStamped()
            temp.pose.position = transform.transform.translation
            temp.pose.orientation.w = 1.0   

            position = tf2_geometry_msgs.do_transform_pose(temp, transform2) #frame_id = odom child_frame: baselink
            position.pose.position.z = 0.0
            
            self.publish_pose(position.pose.position, position.pose.orientation, self.current_ori, msg.header.stamp)
#            self.broadcast_aruco(position.pose.position, position.pose.orientation, msg.header.stamp)

    def current_pose_callback(self, msg):
        self.current_ori = msg.pose.pose.orientation

    def publish_pose(self, translation, quaternion, current_ori, time):
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

#        msg.pose.pose.orientation = current_ori
#        self.set_pose_pub.publish(msg)
#        self.global_set_pose_pub.publish(msg)
        

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
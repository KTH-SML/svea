#! /usr/bin/env python3

import numpy as np

import rospy

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from aruco_msgs.msg import Marker

class static_svea_gps:

    def __init__(self):
        #initalize the node 
        rospy.init_node('static_svea_gps')

        #Subscriber
        rospy.Subscriber("static/gps/fix", NavSatFix, self.gps_callback) # gps signal (lat long) of static svea
        rospy.Subscriber("static/odometry/gps", TFMessage, self.odom_gps_callback) # position of static svea
        rospy.Subscriber("aruco_pose", Marker, self.aruco_callback)
        
        #Publisher
        self.pose_pub = rospy.Publisher("static/pose", PoseWithCovarianceStamped, queue_size=10) #publish to pose0 in ekf
        
        #Variable
        self.gps_msg = None
        self.location = None
        self.aruco_pose = None

        #Transformation
        self.listener = tf.TransformListener()

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
        #inverse ?
        self.aruco_header = msg.header
        self.aruco_pose = msg.pose
        self.inverse_aruco()

    def publish_pose(self):
        self.pose_pub.publish(self.location)

    def inverse_aruco(self):
        try:
            (trans, rot) = self.listener.lookupTransform('camera', 'aruco11', rospy.Time(0))
            try:
                transform_matrix = tf.transformations.concatenate_matrices(
                    tf.transformations.translation_matrix(trans),
                    tf.transformations.quaternion_matrix(rot))
                try:
                    inversed_transform = tf.transformations.inverse_matrix(transform_matrix)
                    try:
                        translation = tf.transformations.translation_from_matrix(inversed_transform)
                        quaternion = tf.transformations.quaternion_from_matrix(inversed_transform)
                        print("translation", translation)
                        print("=================================================")
                        print("quaternion", quaternion)
                    except:
                        rospy.loginfo("Cannot get translation/ quaternion from inversed matrix")                        
                except:
                    rospy.loginfo("Cannot inverse matrix")
            except:
                rospy.loginfo("Cannot concatenate matrices")
        except:
            rospy.loginfo("Cannot lookup transform")
            

if __name__ == '__main__':
    static_svea_gps().run()
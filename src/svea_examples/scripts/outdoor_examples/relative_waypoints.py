#!/usr/bin/env python3

import math
import numpy as np 
from itertools import chain

from geopy import distance as geodistnace

import matplotlib.pyplot as plt

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import euler_from_quaternion #, quaternion_from_euler

########################################################################################
###Transform the gps location into map frame (x,y) based on the starting pose of SVEA###
########################################################################################

class relative_waypoints():
    # The coordinates below are 4 points inside the carpark outside of ITRL
    corners = [[59.3508586, 18.0679122],
               [59.3508938, 18.0680175],
               [59.3508563, 18.0680577],
               [59.3508059, 18.0679789]]

    def __init__(self):
        # Initalize the node 
        rospy.init_node("relative_waypoints", anonymous=False)

        self.waypoint_topic = rospy.get_param("~waypoints_topic", "/outdoor_localization_waypoint")
        self.default_corners = rospy.get_param("~corners", self.corners)
        self.resolution = rospy.get_param("~resolution", 10)
        self.location_topic = rospy.get_param("~location_topic", "/gps/filtered")
        self.vis_topic = rospy.get_param("~marker_topic", "/waypoints")
        self.gps_odometry_topic = rospy.get_param("~gps_odometry_topic", "/odometry/filtered/global")

        # Subscriber
        rospy.Subscriber(self.location_topic, NavSatFix, self.starting_location_callback)
        rospy.Subscriber(self.gps_odometry_topic, Odometry, self.gps_odometry_callback)

        # Publisher
        self.pub_waypoints = rospy.Publisher(self.waypoint_topic, Float64MultiArray, queue_size=10)
        self.pub_visualization = rospy.Publisher(self.vis_topic, MarkerArray, queue_size=10)
        
        # Variable
        self.start = None
        self.coor = None
        self.starting_ori = None
        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown() and (self.coor is None or self.start is None):
            self.rate.sleep()

        self.generate_waypoints()

    def run(self):
        rospy.spin()

    def gps_odometry_callback(self,msg):
        if self.coor is None:
            self.coor = [[msg.pose.pose.position.x, msg.pose.pose.position.y]]
            roll, pitch, self.starting_ori = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    def starting_location_callback(self, msg):
        if self.start is None:
            #self.start = [59.3507979, 18.0679703]
            self.start = [msg.latitude, msg.longitude]
            print(self.start)

    def generate_waypoints(self):
        default_corners_r = np.radians(self.default_corners)
        start_r = np.radians(self.start)
        for count, point in enumerate(default_corners_r):
            y = np.sin(point[1]-start_r[1])*np.cos(point[0])
            x = np.cos(start_r[0])*np.sin(point[0])-np.sin(start_r[0])*np.cos(point[0])*np.cos(point[1]-start_r[1])
            brng = np.pi/2 - np.arctan2(y, x) + self.starting_ori
            distance = geodistnace.geodesic(np.degrees(point), self.start, ellipsoid='GRS-80').m
            coor_x = distance * np.cos(brng) + self.coor[0][0]
            coor_y = distance * np.sin(brng) + self.coor[0][1]

            self.coor.append([coor_x, coor_y])
            print(brng, distance, self.coor[-1])
    
        pub_points = Float64MultiArray()
        pub_points.data = list(chain.from_iterable(self.coor[1:]))
        self.visualize()
        self.pub_waypoints.publish(pub_points)


    def visualize(self):
        marker_array = MarkerArray()
        green = 0.2
        for id, point in enumerate(self.coor):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = id
            marker.type = Marker().SPHERE
            marker.action = Marker().ADD
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = green
            marker.color.b = 0.0
            marker_array.markers.append(marker)
            green += 0.2

        self.pub_visualization.publish(marker_array)

if __name__ == '__main__':
    relative_waypoints().run()

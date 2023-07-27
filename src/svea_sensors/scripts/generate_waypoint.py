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
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#import plotly.graph_objects as go

class generate():
    #lat, long
    '''#    default_corners = [[59.35090600000875, 18.068049400004035], 
    #                [59.35087019986695, 18.067948899970027] ,
    #                [59.350880999981385, 18.067930400101968] ,
    #                [59.35091747278197, 18.06802886626117]]

    #    default_corners = [[59.35090538359178,18.068052862349326], 
    #                [59.35088159984089, 18.06798270093596] ,
    #                [59.350893398289955, 18.06796449531501] ,
    #                [59.35091946020317, 18.068035701770267]]

    #    default_corners = [[59.3508205 , 18.0679565],
    #                       [59.350855, 18.0680513], 
    #                       [59.350878, 18.0680259],
    #                       [59.3508457, 18.0679246]] #carpark
    #    default_corners = [[59.350703089697824, 18.067884361913592],
    #                       [59.35066237301757, 18.067937795070453],
    #                       [59.35067459897018, 18.067979029770026],
    #                       [59.35072082906835, 18.06792115256776]] #padestrian

    #    default_corners = [[59.350712, 18.0678711],
    #                    [59.3506577, 18.0679434]] # straight line on padertrain

    #    default_corners = [[59.3507694, 18.0678011],
    #                    [59.3507132, 18.0678728]] # straight line on padertrain
        
    #    default_corners = [[59.3507715, 18.0677963],
    #                    [59.3507224, 18.0678608]] # straight line on padertrain

    #59.3507331
    #longitude: 18.0678454


    #59.3507141,18.067871
    #59.3506717
    #longitude: 18.0679268'''
    
#    default_corners = [[59.3508579, 18.0679108],
#                       [59.3508927, 18.0680166],
#                       [59.3508488, 18.0680664],
#                        [59.3508068, 18.0679704]] #carpark2
    
    default_corners = [[59.3508586, 18.0679122],
                       [59.3508938, 18.0680175],
                       [59.3508563, 18.0680577],
                        [59.3508059, 18.0679789]] #carpark3

#    default_corners = [[59.3508059, 18.0679789],
#                       [59.3508563, 18.0680577],
#                       [59.3508938, 18.0680175],
#                       [59.3508586, 18.0679122]] #carpark3 revrese

#    default_corners = [[59.3508586, 18.0679122],
#                       [59.3508938, 18.0680175]] #carpark3 first 2 points
    
    def __init__(self):

        self.waypoint_topic = rospy.get_param("~waypoints_topic", "/outdoor_localization_waypoint")
        self.corners = rospy.get_param("~corners", self.default_corners)
        self.resolution = rospy.get_param("~resolution", 10)
        self.location_topic = rospy.get_param("~location_topic", "/gps/filtered")
        self.vis_topic = rospy.get_param("~marker_topic", "/waypoints")
        self.gps_odometry_topic = rospy.get_param("~gps_odometry_topic", "/odometry/filtered/global")

        #Subscriber
        rospy.Subscriber(self.location_topic, NavSatFix, self.starting_location_callback)
        rospy.Subscriber(self.gps_odometry_topic, Odometry, self.gps_odometry_callback)

        #Publisher
        self.pub_waypoints = rospy.Publisher(self.waypoint_topic, Float64MultiArray, queue_size=10)
        self.pub_visualization = rospy.Publisher(self.vis_topic, MarkerArray, queue_size=10)
        #params
        self.start = None
        self.coor = None
        self.starting_ori = None
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown() and (self.coor is None or self.start is None):
            self.rate.sleep()

        self.generate_waypoints()

    def gps_odometry_callback(self,msg):
        if self.coor is None:
            self.coor = [[msg.pose.pose.position.x, msg.pose.pose.position.y]]
            roll, pitch, self.starting_ori = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    def starting_location_callback(self, msg):
        if self.start is None:
            #self.start = [59.3507979, 18.0679703]
            self.start = [msg.latitude, msg.longitude]
            print(self.start)

    '''def show_on_map(self):
        dc_array = np.array(self.default_corners)
        color = [
                    '#1f77b4',  # muted blue
                    '#ff7f0e',  # safety orange
                    '#2ca02c',  # cooked asparagus green
                    '#d62728',  # brick red
                    '#9467bd',  # muted purple
                    '#8c564b',  # chestnut brown
                    '#e377c2',  # raspberry yogurt pink
                    '#7f7f7f',  # middle gray
                    '#bcbd22',  # curry yellow-green
                    '#17becf'   # blue-teal
                ]
        
        for count, point in enumerate(dc_array):
            if count == 0:
                fig = go.Figure(go.Scattermapbox(
                    name = "point" + str(count),
                    fillcolor = color[count%len(color)],
                    mode = "markers",
                    lon = [point[1]],
                    lat = [point[0]],
                    marker = {'size': 10}))
            else:
                fig.add_trace(go.Scattermapbox(
                    name = "point" + str(count),
                    fillcolor = color[count%len(color)],
                    mode = "markers",
                    lon = [point[1]],
                    lat = [point[0]],
                    marker = {'size': 10}))

        fig.update_layout(
            margin ={'l':0,'t':0,'b':0,'r':0},
            mapbox = {
                'style': "open-street-map",
                'center': {'lon': 18.0680513, 'lat': 59.350855},
                'zoom': 18  })
        
        fig.show()'''

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
            #self.coor.append(temp[count])
            print(brng, distance, self.coor[-1])
    
        #working
        '''for count, point in enumerate(default_corners_r):
            y = np.sin(point[1]-default_corners_r[0][1])*np.cos(point[0])
            x = np.cos(default_corners_r[0][0])*np.sin(point[0])-np.sin(default_corners_r[0][0])*np.cos(point[0])*np.cos(point[1]-default_corners_r[0][1])
            brng = np.pi/2 - np.arctan2(y, x)
            distance = geodistnace.geodesic(np.degrees(point), np.degrees(default_corners_r[0]), ellipsoid='GRS-80').m
            coor_x = distance * np.cos(brng)
            coor_y = distance * np.sin(brng)

            self.coor.append([coor_x, coor_y])
            print(brng, distance, self.coor[-1])'''
        
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
            marker.pose.position.z = 0
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
#    coor_array = np.array(coor)
#    color_list = ['b','g','r','c','m','y','k','w']
#    for count1, coor_ in enumerate(coor_array):
#        plt.plot(coor_[0], coor_[1], 'o', color=color_list[count1])
#    plt.show()

if __name__ == '__main__':
    rospy.init_node("generate_waypoints", anonymous=False)
    rate = rospy.Rate(10)
    generat_points = generate()
    while not rospy.is_shutdown():
        rate.sleep()

#!/usr/bin/env python3

import math
import rospy
import numpy as np 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
from sensor_msgs.msg import NavSatFix
import pandas as pd

import plotly.graph_objects as go

class plot_localization():
    def __init__(self):
        self.plot_msg_odom_top = rospy.get_param("~plot_msg_topic", "/odometry/filtered/global")
        self.plot_msg_gps_top = rospy.get_param("~plot_msg_topic", "/gps/filtered")
    
        #subscriber
        rospy.Subscriber(self.plot_msg_odom_top, Odometry, self.plot_msg_odom_callback)
        rospy.Subscriber(self.plot_msg_gps_top, NavSatFix, self.plot_msg_gps_callback)

        #parameter
        self.fig, self.ax = plt.subplots(2,1)

        self.pose_odom_x = []
        self.pose_odom_y = []
        self.lat  = []
        self.long = []

        self.counter = 0

    def plot_msg_odom_callback(self, msg):
        self.pose_odom_x.append(msg.pose.pose.position.x)
        self.pose_odom_y.append(msg.pose.pose.position.y) 

    def plot_msg_gps_callback(self, msg):
        self.counter += 1
        self.lat.append(msg.latitude)
        self.long.append(msg.longitude)
        
    def plot_map(self):
        self.counter += 1
        self.ax[0].plot(self.pose_odom_y, self.pose_odom_x, '*', color='blue')
        self.ax[1].plot(self.lat, self.long, '*', color='red')
        plt.draw()
        plt.pause(0.00000000001)

    def final_plot(self):
        fig = go.Figure(go.Scattermapbox(
            mode = "markers",
            lon = self.long,
            lat = self.lat,
            marker = {'size': 10}))

        fig.add_trace(go.Scattermapbox(
            mode = "markers",
            lon = self.long,
            lat = self.lat,
            marker = {'size': 10}))

        fig.update_layout(
                margin ={'l':10,'t':10,'b':10,'r':10},
                mapbox = {
                    'center': {'lon': 18.07, 'lat': 59.35},
                    'style': "open-street-map",
                    'zoom': 15
                    })

        fig.show()
if __name__ == '__main__':
    rospy.init_node('plot_localization', anonymous=False)
    time_start = rospy.Time.now()
    rate = rospy.Rate(10)
    plot_obj = plot_localization()
    last_counter = -1
    entered = False
    while not rospy.is_shutdown():
        plot_obj.plot_map()
        rate.sleep()
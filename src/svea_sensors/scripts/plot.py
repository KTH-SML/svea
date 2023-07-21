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
        
        #plt.xlim([-5, -16])
        #plt.ylim([-25, -5])

    def plot_msg_gps_callback(self, msg):
        self.counter += 1
        self.lat.append(msg.latitude)
        self.long.append(msg.longitude)
        
    def plot_map(self):
        self.counter += 1
        self.ax[0].plot(self.pose_odom_y, self.pose_odom_x, '*', color='blue')
        #self.ax[0].set_xlim([-5, -16])
        #self.ax[0].set_ylim([-25, -5])
        self.ax[1].plot(self.lat, self.long, '*', color='red')
        #self.ax[1].set_ylim([18.0679, 18.0682])
        #self.ax[1].set_xlim([59.35076, 59.35088])
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
            margin ={'l':0,'t':0,'b':0,'r':0},
            mapbox = {
                'center': {'lon': 10, 'lat': 10},
                'style': "open-street-map",
                'center': {'lon': -20, 'lat': -20},
                'zoom': 1})

        fig.show()
if __name__ == '__main__':
    rospy.init_node('plot_outdoor_localization', anonymous=False)
    time_start = rospy.Time.now()
    rate = rospy.Rate(10)
    plot_obj = plot_localization()
    last_counter = -1
    entered = False
    same = 0
    while not rospy.is_shutdown():
        plot_obj.plot_map()
        if abs(last_counter-plot_obj.counter) == 1:
            same += 1
        else:
            same = 0
        print((rospy.Time.now() - time_start).to_sec())
        if (rospy.Time.now() - time_start).to_sec() > 7 and not entered:
            plot_obj.final_plot()
            entered = True
            df = pd.DataFrame({"lat" : plot_obj.lat, "long" : plot_obj.long})
            df.to_csv("auto_7_21_8.csv", index=False)
        else:
            last_counter = plot_obj.counter
        rate.sleep()
#        if same == 20:
#            break
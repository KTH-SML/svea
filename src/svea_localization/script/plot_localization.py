#!/usr/bin/env python3

import math
import rclpy
import numpy as np 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
from sensor_msgs.msg import NavSatFix

import plotly.graph_objects as go
from rclpy.node import Node  # Add this import

class plot_localization(Node):  # Inherit from Node
    def __init__(self):
        super().__init__('plot_localization')  # Initialize the Node
        self.declare_parameter("~plot_msg_topic", "/odometry/filtered/global")
        self.plot_msg_odom_top = self.get_parameter("~plot_msg_topic").value

        self.declare_parameter("~plot_msg_gps_topic", "/gps/filtered")
        self.plot_msg_gps_top = self.get_parameter("~plot_msg_gps_topic").value
    
        #subscriber
        self.create_subscription(NavSatFix, self.plot_msg_gps_top, self.plot_msg_gps_callback, 10)
        self.create_subscription(NavSatFix, self.plot_msg_gps_top, self.plot_msg_gps_callback, 10)

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

def main():
    rclpy.init()
    plot_obj = plot_localization()
    last_counter = -1
    entered = False
    while rclpy.ok():
        plot_obj.plot_map()
        rclpy.spin_once(plot_obj, timeout_sec=0.1)
    
if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import math
import numpy as np 
import matplotlib.pyplot as plt
import pandas as pd

import plotly.express as px
import plotly.graph_objects as go

def plot91011():
#    lat_long_csv9 = '/home/annika/svea_rtk_navsat/rosbag/test9_gps_filtered.csv'
#    lat_long_csv10 = '/home/annika/svea_rtk_navsat/rosbag/test10_gps_filtered.csv'
#    lat_long_csv11 = '/home/annika/svea_rtk_navsat/rosbag/test11_gps_filtered.csv'
#    lat_long_csv12_1 = '/home/annika/svea_rtk_navsat/rosbag/test12_1_gps_filtered.csv'
#    lat_long_csv12_2 = '/home/annika/svea_rtk_navsat/rosbag/test12_2_gps_filtered.csv'
#
#    data12_1= pd.read_csv(lat_long_csv12_1)
#    data12_2= pd.read_csv(lat_long_csv12_2)
#    data9 = pd.read_csv(lat_long_csv9)
#    data10 = pd.read_csv(lat_long_csv10)
#    data11 = pd.read_csv(lat_long_csv11)
    plot_pt = []
    lat_long_csv7_20 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_20.csv'
    lat_long_csv7_21 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_21.csv'
    lat_long_csv7_21_2 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_21_2.csv'
    lat_long_csv7_21_3 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_21_3.csv'
    lat_long_csv7_21_4 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_21_4.csv'
    lat_long_csv7_21_5 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_21_5.csv'
    lat_long_csv7_21_6 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_21_6.csv'
    data11 = pd.read_csv(lat_long_csv7_20)
    data12 = pd.read_csv(lat_long_csv7_21)
    data13 = pd.read_csv(lat_long_csv7_21_2)
    data14 = pd.read_csv(lat_long_csv7_21_3)
    data15 = pd.read_csv(lat_long_csv7_21_4)
    data16 = pd.read_csv(lat_long_csv7_21_5)
    data17 = pd.read_csv(lat_long_csv7_21_6)

    default_corners = [[59.3508068, 18.0679704],
                    [59.3508488, 18.0680664], 
                    [59.3508927, 18.0680166],
                    [59.3508579, 18.0679108]] #carpark2
    target = np.array(default_corners)
    fig = px.scatter_geo(data11, lat='lat', lon='long')

    fig = go.Figure(go.Scattermapbox(
        name= "11",
        mode = "markers",
        lon = data11.long,
        lat = data11.lat,
        marker = {'size': 3}))


    fig.add_trace(go.Scattermapbox(
        name= "12",
        mode = "markers",
        lon = data12.long,
        lat = data12.lat,
        marker = {'size': 3}))
    
    fig.add_trace(go.Scattermapbox(
        name= "13",
        mode = "markers",
        lon = data13.long,
        lat = data13.lat,
        marker = {'size': 3}))
    
    fig.add_trace(go.Scattermapbox(
        name= "14",
        mode = "markers",
        lon = data14.long,
        lat = data14.lat,
        marker = {'size': 3}))

    fig.add_trace(go.Scattermapbox(
        name= "target",
        mode = "markers",
        lon = target[:,1],
        lat = target[:,0],
        marker = {'size': 10}))

    fig.add_trace(go.Scattermapbox(
        name= "15",
        mode = "markers",
        lon = data15.long,
        lat = data15.lat,
        marker = {'size': 3}))
    
    fig.add_trace(go.Scattermapbox(
        name= "16",
        mode = "markers",
        lon = data16.long,
        lat = data16.lat,
        marker = {'size': 3}))
    
    fig.add_trace(go.Scattermapbox(
        name= "self drive",
        mode = "markers",
        lon = data17.long,
        lat = data17.lat,
        marker = {'size': 3}))
    
    fig.update_layout(
        margin ={'l':10,'t':10,'b':10,'r':10},
        mapbox = {
            'center': {'lon': 18.07, 'lat': 59.35},
            'style': "open-street-map",
            'zoom': 15
            })

    fig.show()

def plot12():
    lat_long_csv12_1 = '/home/annika/svea_rtk_navsat/rosbag/test12_1_gps_filtered.csv'
    lat_long_csv12_2 = '/home/annika/svea_rtk_navsat/rosbag/test12_2_gps_filtered.csv'

    data12_1= pd.read_csv(lat_long_csv12_1)
    data12_2= pd.read_csv(lat_long_csv12_2)
    
    fig = px.scatter_geo(data12_1, lat='lat', lon='long')

    fig = go.Figure(go.Scattermapbox(
        name= "along pedestrian path",
        mode = "markers",
        lon = data12_1.long,
        lat = data12_1.lat,
        marker = {'size': 3}))

    fig.add_trace(go.Scattermapbox(
        name= "along the edge",
        mode = "markers",
        lon = data12_2.long,
        lat = data12_2.lat,
        marker = {'size': 3}))

    fig.update_layout(
        margin ={'l':0,'t':0,'b':0,'r':0},
        mapbox = {
            'center': {'lon': 18.07, 'lat': 59.35},
            'style': "open-street-map",
            'zoom': 15
            })

    fig.show()

plot91011()
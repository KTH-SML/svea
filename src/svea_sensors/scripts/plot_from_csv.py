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

    lat_long_csv7_20 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_20.csv'
    lat_long_csv7_21 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_21.csv'
    lat_long_csv7_21_2 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_21_2.csv'
    data11 = pd.read_csv(lat_long_csv7_20)
    data12 = pd.read_csv(lat_long_csv7_21)
    data13 = pd.read_csv(lat_long_csv7_21_2)



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
    

    fig.update_layout(
        margin ={'l':0,'t':0,'b':0,'r':0},
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
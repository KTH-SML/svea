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
    lat_long_csv7_21_7 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_21_7.csv'
    lat_long_csv7_21_8 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_21_8.csv'
    lat_long_csv7_24_1 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_24_1.csv'
    lat_long_csv7_24_2 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_24_2.csv'
    lat_long_csv7_27_1 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_27_1.csv'
    lat_long_csv7_27_2 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_27_14_56.csv'
    lat_long_csv7_27_3 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_27_15_55.csv'
    lat_long_csv7_27_4 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_27_16_52.csv'
    lat_long_csv7_27_5 = '/home/annika/svea_rtk_navsat/rosbag/auto_7_27_straight.csv'

    data11 = pd.read_csv(lat_long_csv7_20)
    data12 = pd.read_csv(lat_long_csv7_21)
    data13 = pd.read_csv(lat_long_csv7_21_2)
    data14 = pd.read_csv(lat_long_csv7_21_3)
    data15 = pd.read_csv(lat_long_csv7_21_4)
    data16 = pd.read_csv(lat_long_csv7_21_5)
    data17 = pd.read_csv(lat_long_csv7_21_6)
    data18 = pd.read_csv(lat_long_csv7_21_7)
    data19 = pd.read_csv(lat_long_csv7_21_8)
    data20 = pd.read_csv(lat_long_csv7_24_1)
    data21 = pd.read_csv(lat_long_csv7_24_2)
    data22 = pd.read_csv(lat_long_csv7_27_1)
    data23 = pd.read_csv(lat_long_csv7_27_2)
    data24 = pd.read_csv(lat_long_csv7_27_3)
    data25 = pd.read_csv(lat_long_csv7_27_4)
    data26 = pd.read_csv(lat_long_csv7_27_5)

    default_corners = [[59.3508586, 18.0679122],
                       [59.3508938, 18.0680175],
                       [59.3508563, 18.0680577],
                        [59.3508059, 18.0679789]] #carpark3
    target = np.array(default_corners)
    size = 5
    fig = px.scatter_geo(data11, lat='lat', lon='long')

    fig = go.Figure(go.Scattermapbox(
        name= "target",
        mode = "markers",
        lon = target[:,1],
        lat = target[:,0],
        marker = {'size': 10}))

    '''ig.add_trace(go.Scattermapbox(
        name= "15",
        mode = "markers",
        lon = data15.long,
        lat = data15.lat,
        marker = {'size': size}))
    
    fig.add_trace(go.Scattermapbox(
        name= "16",
        mode = "markers",
        lon = data16.long,
        lat = data16.lat,
        marker = {'size': size}))
    
    fig.add_trace(go.Scattermapbox(
        name= "self drive",
        mode = "markers",
        lon = data17.long,
        lat = data17.lat,
        marker = {'size': size}))
    
    fig.add_trace(go.Scattermapbox(
        name= "18",
        mode = "markers",
        lon = data18.long,
        lat = data18.lat,
        marker = {'size': size}))
    
    fig.add_trace(go.Scattermapbox(
        name= "19",
        mode = "markers",
        lon = data19.long,
        lat = data19.lat,
        marker = {'size': size}))
    
    fig.add_trace(go.Scattermapbox(
        name= "20",
        mode = "markers",
        lon = data20.long,
        lat = data20.lat,
        marker = {'size': size})) 

    fig.add_trace(go.Scattermapbox(
        name= "21",
        mode = "markers",
        lon = data21.long,
        lat = data21.lat,
        marker = {'size': size})) 

    fig.add_trace(go.Scattermapbox(
        name= "22",
        mode = "markers",
        lon = data22.long,
        lat = data22.lat,
        marker = {'size': size})) 

    fig.add_trace(go.Scattermapbox(
        name= "23",
        mode = "markers",
        lon = data23.long,
        lat = data23.lat,
        marker = {'size': size})) '''

    fig.add_trace(go.Scattermapbox(
        name= "24",
        mode = "markers",
        lon = data24.long,
        lat = data24.lat,
        marker = {'size': size})) 
               
    fig.add_trace(go.Scattermapbox(
        name= "25",
        mode = "markers",
        lon = data25.long,
        lat = data25.lat,
        marker = {'size': size})) 

    fig.add_trace(go.Scattermapbox(
        name= "26",
        mode = "markers",
        lon = data26.long,
        lat = data26.lat,
        marker = {'size': size}))     

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
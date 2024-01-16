#!/usr/bin/env python3
import rospy
import tf2_ros

import numpy as np

from geopy.distance import geodesic

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from svea_msgs.msg import VehicleState
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion


class FixedFrameState:
    def __init__(self):
        # Initialize node
        rospy.init_node('FixedFrameState')

        # Subscriber
        rospy.Subscriber('/gps/filtered', NavSatFix, self.gpsFilteredCallback)

        # Publisher
        self.FixedFrameState = rospy.Publisher('/FixedFrameState', VehicleState, queue_size=10)

        # Fixed GPS latitude, Longitude
        self.fixed_gps = (59.350791, 18.067825) #ITRL pose

        # Transformation
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.br = tf2_ros.TransformBroadcaster()

    def run(self):
        rospy.spin()

    def gpsFilteredCallback(self, msg):
        filtered_gps = (msg.latitude, msg.longitude)
        x, y = self.LatLongToXY(filtered_gps)
        self.StateMsg(msg.header, x,y)

    def LatLongToXY(self, filtered_gps):
        distance = geodesic(self.fixed_gps, filtered_gps).meters

        bearing = self.CalculateBearing(filtered_gps)

        # Convert bearing and distance to x, y coordinates
        x = distance * np.sin(np.radians(bearing))
        y = distance * np.cos(np.radians(bearing))

        return x, y

    def CalculateBearing(self, filtered_gps):
        lat1 = np.radians(self.fixed_gps[0])
        lat2 = np.radians(filtered_gps[0])
        diffLong = np.radians(filtered_gps[1] - self.fixed_gps[1])

        x = np.sin(diffLong) * np.cos(lat2)
        y = np.cos(lat1) * np.sin(lat2) - (np.sin(lat1) * np.cos(lat2) * np.cos(diffLong))

        initial_bearing = np.arctan2(x, y)

        initial_bearing = np.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360

        return compass_bearing
    
    def StateMsg(self, header, x, y):
        msg = VehicleState()
        msg.header = header
        msg.child_frame_id = "base_link_fixed"
        msg.x = x
        msg.y = y
        msg.yaw = 0 
        msg.v = 0
        msg.covariance = [0, 0, 0, 0,
                          0, 0, 0, 0,
                          0, 0, 0, 0,
                          0, 0, 0, 0]
        self.FixedFrameState.publish(msg)

        msgT = TransformStamped()
        msgT.header = header
        msgT.child_frame_id = "base_link_fixed"
        msgT.transform.translation = Vector3(*[x,y,0.0])
        msgT.transform.rotation = Quaternion(*[0.0, 0.0, 0.0, 1.0])
        self.br.sendTransform(msgT)
if __name__ == '__main__':
    FixedFrameState().run()

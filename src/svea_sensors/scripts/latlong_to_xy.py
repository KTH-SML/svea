#!/usr/bin/env python3
import rospy
import tf2_ros

import numpy as np

from geopy.distance import geodesic

from sensor_msgs.msg import NavSatFix
from svea_msgs.msg import VehicleState
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion

class latlong_to_xy:
    def __init__(self):
        # Initialize node
        rospy.init_node('latlong_to_xy')

        self.reference_gps = (-1, -1)

        # Subscriber
        rospy.Subscriber('/gps/filtered', NavSatFix, self.vehicle_gps_callback)
        rospy.Subscriber('/gps/reference_point', NavSatFix, self.reference_gps_callback)

        # Publisher
        self.RefernceGpsState = rospy.Publisher('/gps/reference_state', VehicleState, queue_size=10)

        # Transformation
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()

    def run(self):
        rospy.spin()

    def reference_gps_callback(self, msg):
        if -1 in self.reference_gps:
            self.reference_gps = (msg.latitude, msg.longitude)

    def vehicle_gps_callback(self, msg):
        if not -1 in self.reference_gps:
            x, y = self.LatLongToXY((msg.latitude, msg.longitude))
            self.StateMsg(msg.header, x,y)

    def LatLongToXY(self, vehicle_gps):
        distance = geodesic(self.reference_gps, vehicle_gps).meters

        bearing = self.CalculateBearing(vehicle_gps)

        x = distance * np.sin(np.radians(bearing))
        y = distance * np.cos(np.radians(bearing))

        return x, y

    def CalculateBearing(self, vehicle_gps):
        lat1 = np.radians(self.reference_gps[0])
        lat2 = np.radians(vehicle_gps[0])
        diffLong = np.radians(vehicle_gps[1] - self.reference_gps[1])

        x = np.sin(diffLong) * np.cos(lat2)
        y = np.cos(lat1) * np.sin(lat2) - (np.sin(lat1) * np.cos(lat2) * np.cos(diffLong))

        initial_bearing = np.arctan2(x, y)

        initial_bearing = np.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360

        return compass_bearing
    
    def StateMsg(self, header, x, y):
        msg = VehicleState()
        msg.header = header
        msg.header.frame_id = "reference_gps"
        msg.child_frame_id = "base_link_fixed_gps"
        msg.x = x
        msg.y = y
        msg.yaw = 0 
        msg.v = 0
        msg.covariance = [0, 0, 0, 0,
                          0, 0, 0, 0,
                          0, 0, 0, 0,
                          0, 0, 0, 0]
        self.RefernceGpsState.publish(msg)

        msgT = TransformStamped()
        msgT.header = header
        msgT.header.frame_id = "reference_gps"
        msgT.child_frame_id = "base_link_fixed_gps"
        msgT.transform.translation = Vector3(*[x,y,0.0])
        msgT.transform.rotation = Quaternion(*[0.0, 0.0, 0.0, 1.0])
        self.br.sendTransform(msgT)

if __name__ == '__main__':
    latlong_to_xy().run()

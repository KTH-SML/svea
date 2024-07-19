#!/usr/bin/env python3

import rospy
from tf.transformations import quaternion_from_euler
from robot_localization.srv import SetDatum
from geographic_msgs.msg import GeoPose

import os
import ast
import yaml


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


class SetDatumNode:
    def __init__(self) -> None:
        try:
            # Initialize node
            rospy.init_node('set_datum_node')
            
            # Topic Parameters
            self.datum_service = load_param('~datum_service', 'datum')
            
            # Other Parameters
            self.datum_file = load_param('~datum_file', "")
            self.datum_data = load_param('~datum_data', "[]")
            self.service_timeout = load_param('~service_timeout', 60)
            try:
                self.datum_data = ast.literal_eval(self.datum_data)
            except Exception as e:
                raise ValueError(f"{rospy.get_name()}: Invalid datum_data provided, it should be a list of 3 elements. Error: {e}")
            
            # Warning if both datum_file and datum_data are provided
            if self.datum_file != "" and len(self.datum_data) > 0:
                rospy.logwarn(f"{rospy.get_name()}: Both datum_file and datum_data are provided. datum_file will be used and datum_data will be ignored.")
                
            # Get datum from parameters
            if self.datum_file != "":
                self.datum = self.load_datum_from_yaml(self.datum_file)
            elif len(self.datum_data) == 3:
                self.datum = {
                    'latitude': self.datum_data[0],
                    'longitude': self.datum_data[1],
                    'yaw': self.datum_data[2]
                }
            else:
                raise ValueError(f"{rospy.get_name()}: No valid datum provided. Please provide either datum_file or datum_data with latitude, longitude, and yaw.")      
            
            # Service
            rospy.loginfo(f"{rospy.get_name()} Waiting for service {self.datum_service} for max {self.service_timeout} seconds")
            rospy.wait_for_service(self.datum_service, timeout=self.service_timeout)
            rospy.loginfo(f"{rospy.get_name()} Service {self.datum_service} found, continuing...")
            self.set_datum_service = rospy.ServiceProxy(self.datum_service, SetDatum)
        
        except Exception as e:
            # Log error
            rospy.logerr(e)
            # Shutdown node
            rospy.signal_shutdown(f"{rospy.get_name()}: Initialization failed.")

        else:
            # Log status
            rospy.loginfo(f"{rospy.get_name()}: Initialized successfully.")
            # Set datum
            self.set_datum()

    def load_datum_from_yaml(self, file_path):
        if os.path.exists(file_path):
            with open(file_path, 'r') as file:
                datum_data = yaml.safe_load(file)
            return datum_data['datum']
        else:
            rospy.logerr(f"{rospy.get_name()}: File {file_path} does not exist, shutting down the node.")
            rospy.signal_shutdown(f"{rospy.get_name()}: File {file_path} does not exist.")

    def set_datum(self):
        # Set datum service request
        orientation = quaternion_from_euler(0, 0, self.datum['yaw'])
        geopose = GeoPose()
        geopose.position.latitude = self.datum['latitude']
        geopose.position.longitude = self.datum['longitude']
        geopose.position.altitude = 0.0
        geopose.orientation.x = orientation[0]
        geopose.orientation.y = orientation[1]
        geopose.orientation.z = orientation[2]
        geopose.orientation.w = orientation[3]
        
        # Call set datum service
        try:
            response = self.set_datum_service(geopose)
            if response:
                rospy.loginfo(f"{rospy.get_name()}: Datum set successfully.")
            else:
                rospy.logerr(f"{rospy.get_name()}: Failed to set datum.")
        except rospy.ServiceException as e:
            rospy.logerr(f"{rospy.get_name()}: Service call failed: {e}")
            
            
if __name__ == '__main__':
    node = SetDatumNode()
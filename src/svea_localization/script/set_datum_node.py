#!/usr/bin/env python3

__author__ = "Sulthan Suresh Fazeela"
__email__ = "sultha@kth.se"
__license__ = "MIT"

import rclpy
import rclpy.exceptions
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from robot_localization.srv import SetDatum
from geographic_msgs.msg import GeoPose

import os
import ast
import yaml


def load_param(self, name, value=None):
    self.declare_parameter(name, value)
    if value is None:
        assert self.has_parameter(name), f'Missing parameter "{name}"'
    return self.get_parameter(name).value


class SetDatumNode(Node):
    def __init__(self) -> None:
        try:
            # Initialize node
            super().__init__('set_datum_node')
            
            # Topic Parameters
            self.datum_service = load_param(self,'~datum_service', 'datum')
            
            # Other Parameters
            self.datum_file = load_param(self,'~datum_file', "")
            self.datum_data = load_param(self,'~datum_data', "[]")
            self.service_timeout = load_param(self,'~service_timeout', 60)
            try:
                self.datum_data = ast.literal_eval(self.datum_data)
            except Exception as e:
                raise ValueError(f"{self.get_name()}: Invalid datum_data provided, it should be a list of 3 elements. Error: {str(e)}")
            
            # Warning if both datum_file and datum_data are provided
            if self.datum_file != "" and len(self.datum_data) > 0:
                self.get_logger().warn(f"{self.get_name()}: Both datum_file and datum_data are provided. datum_file will be used and datum_data will be ignored.")        

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
                raise ValueError(f"{self.get_name()}: No valid datum provided. Please provide either datum_file or datum_data with latitude, longitude, and yaw.")      
            
            # Service
            self.get_logger().info(f"{self.get_name()} Waiting for service {self.datum_service} for max {self.service_timeout} seconds")

            self.client = self.create_client(SetDatum, self.datum_service)

            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service not available, waiting again...')
            
            self.get_logger().info(f"{self.get_name()} Service {self.datum_service} found, continuing...")
        
        except Exception as e:
            super().__init__('set_datum_node')
            # Log error
            self.get_logger().error(str(e))
            # Shutdown node
            self.get_logger().error(f"{self.get_name()}: Initialization failed.")
            rclpy.shutdown()
            
        else:
            super().__init__('set_datum_node')
            # Log status
            self.get_logger().info(f"{self.get_name()}: Initialized successfully.")
            # Set datum
            self.set_datum()

    

    def load_datum_from_yaml(self, file_path):
        if os.path.exists(file_path):
            with open(file_path, 'r') as file:
                datum_data = yaml.safe_load(file)
            return datum_data['datum']
        else:
            self.get_logger().error(f"{self.get_name()}: File {file_path} does not exist, shutting down the node.")
            rclpy.shutdown()

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
                self.get_logger().info(f"{self.get_name()}: Datum set successfully.")
            else:
                self.get_logger().error(f"{self.get_name()}: Failed to set datum.")
        except Exception as e:
            self.get_logger().error(f"{self.get_name()}: Failed to set datum. Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SetDatumNode()

            
if __name__ == '__main__':
    main()
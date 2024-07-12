#!/usr/bin/env python

import rospy
import yaml
import os
import rospkg
from robot_localization.srv import SetDatum
from geographic_msgs.msg import GeoPose

def load_datum_from_yaml(file_path):
    with open(file_path, 'r') as file:
        datum_data = yaml.safe_load(file)
    return datum_data

def set_datum(datum_data):
    rospy.init_node('set_datum_node')

    rospy.wait_for_service('/set_datum')
    try:
        set_datum_service = rospy.ServiceProxy('/set_datum', SetDatum)

        geo_pose = GeoPose()
        geo_pose.position.latitude = datum_data['datum']['position']['latitude']
        geo_pose.position.longitude = datum_data['datum']['position']['longitude']
        geo_pose.position.altitude = datum_data['datum']['position']['altitude']

        geo_pose.orientation.x = datum_data['datum']['orientation']['x']
        geo_pose.orientation.y = datum_data['datum']['orientation']['y']
        geo_pose.orientation.z = datum_data['datum']['orientation']['z']
        geo_pose.orientation.w = datum_data['datum']['orientation']['w']

        response = set_datum_service(geo_pose)
        if response:
            rospy.loginfo("Datum set successfully")
        else:
            rospy.logerr("Failed to set datum")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('set_datum_node')
    
    package_name = rospy.get_param('~package_name', 'your_package_name')
    config_path = rospy.get_param('~datum_config', 'config/datum.yaml')
    
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path(package_name)
    file_path = os.path.join(pkg_path, config_path)
    
    datum_data = load_datum_from_yaml(file_path)
    set_datum(datum_data)

#!/usr/bin/env python3

import rospy
import yaml
import os
from geometry_msgs.msg import PointStamped


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


class PointCollector:
    """
    A ROS node that collects and stores 3D points from the /clicked_point topic.

    This node listens to the /clicked_point topic where messages of type geometry_msgs/PointStamped are published.
    Each received point is stored in an internal list. The node groups the points into sets based on the parameter
    `points_per_person`, which determines the number of consecutive points to group together.

    Features:
    - Collects points and stores them in a list.
    - Groups points into sets of size defined by `points_per_person`.
    - Discards any leftover points that do not form a complete group.
    - Saves the grouped points to a YAML file upon node shutdown.
    - Each group define intermediate points of 1 pedestrian/object trajectory

    Parameters:
    - `file_name` (string): The name of the YAML file where the collected points will be saved. Default is 'trajectories.yaml'.
    - `points_per_person` (int): The number of consecutive points to group together. Default is 2, representing starting and ending positions.
    """
    def __init__(self):
        rospy.init_node('point_collector', anonymous=True)
        self.filename = load_param('~file_name', 'trajectories.yaml')
        self.points_per_person = load_param('~points_per_person', 2)  # default is 2 -> starting position and ending position
        self.points = []
        self.counter = 0
        
        # Set up the subscriber to the /clicked_point topic
        self.subscriber = rospy.Subscriber('/clicked_point', PointStamped, self.callback)
        
        # Register shutdown hook
        rospy.on_shutdown(self.save_points_to_yaml)

    def callback(self, msg):
        # Store the received point in the array and increment counter
        self.points.append(msg.point)
        self.counter += 1
        rospy.loginfo("Received point: (%f, %f, %f)", msg.point.x, msg.point.y, msg.point.z)

    def save_points_to_yaml(self):
        # Group points into sets of points_per_person
        grouped_points = []
        while len(self.points) >= self.points_per_person:
            # Take the next `points_per_person` points
            group = self.points[:self.points_per_person]
            # Remove these points from the list
            self.points = self.points[self.points_per_person:]
            # Add the group to the list of grouped points
            grouped_points.append(group)

        # If there are any remaining points that don't form a complete group, discard them

        # Convert points to the desired YAML format
        yaml_data = {'pedestrians': [[{'x': p.x, 'y': p.y, 'z': p.z} for p in group] for group in grouped_points]}

        # Define the directory and filename
        save_directory = '/svea_ws/src/svea_navigation/maps/simulated_trajectories/'
        full_path = os.path.join(save_directory, self.filename)

        # Ensure the directory exists
        if not os.path.exists(save_directory):
            os.makedirs(save_directory)

        # Save points to YAML file
        with open(full_path, 'w') as file:
            yaml.dump(yaml_data, file, default_flow_style=False)
        
        rospy.loginfo("Points saved to %s", full_path)

if __name__ == '__main__':
    try:
        point_collector = PointCollector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
import yaml
from svea_vision_msgs.msg import StampedObjectPoseArray, ObjectPose
from std_msgs.msg import Header
import numpy as np
import os
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

class ObjectPoseSimulator:
    """
    A ROS node that simulates object poses node (svea_vision) based on trajectories loaded from a YAML file. 
    This simulator is designed to model complex pedestrian trajectories, enabling a quicker tuning on the simulator for navigation.
    It maintains consistency with the message formats and topics used by the real-world 'object_pose' node within the 'svea_vision' package.
    
    This node reads pedestrian intermediete poses (trajectories) from a YAML file and interpolates them linearly.
    The interpolation is based on the desired speed and node publishing frequency, and publishes the poses as StampedObjectPoseArray messages.
    Every time a trajectory is fully published, the simulator republishes it from the beginning.

    The pedestrians' intermediete poses can be created with 'clicked_points_collector.py' before launching the navigation stack.

    Parameters:
    - `yaml_file` (string): Path to the YAML file containing pedestrian trajectories.
    - `speed` (float): Speed of the pedestrians in meters per second. 
    - `frequency` (float): Frequency of the node in Hz.
    """
    def __init__(self, yaml_file, publish_frame_id, speed=0.45, frequency=5.0):
        rospy.init_node('object_pose_node_simulator')
        self.pub = rospy.Publisher('/objectposes', StampedObjectPoseArray, queue_size=1)
        self.yaml_file = yaml_file
        self.speed = speed
        self.frequency = frequency
        self.frame_id = publish_frame_id

        # Load trajectories from YAML file
        self.trajectories = self.load_trajectories()
        self.current_indexes = self.initialize_indexes()
        self.current_positions = []

        # Initialize positions with starting points
        self.initialize_positions()

        # Compute the step size based on speed and frequency
        self.step_size = self.speed / self.frequency

        # Set up the rate for publishing
        self.rate = rospy.Rate(frequency)

        # Initialize TF buffer and listener if needed
        if self.frame_id == 'base_link':
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Start the simulation
        self.simulate_pose_array()

    def load_trajectories(self):
        """
        Load pedestrian trajectories from the YAML file.
        """
        with open(self.yaml_file, 'r') as file:
            data = yaml.safe_load(file)
        return data['pedestrians']
    
    def initialize_indexes(self):
        """
        Initialize the starting indexes.
        """
        initial_indexes = []
        for trajectory in self.trajectories:
            initial_indexes.append(0)  
        return initial_indexes
    
    def initialize_positions(self):
        """
        Initialize the starting positions of the pedestrians from the trajectories.
        """
        for trajectory in self.trajectories:
            if len(trajectory) > 0:
                start_pos = np.array([trajectory[0]['x'], trajectory[0]['y'], trajectory[0]['z']])
                self.current_positions.append(start_pos)
            else:
                self.current_positions.append(np.array([0.0, 0.0, 0.0])) 
    
    def interpolate_position(self, start, end, distance):
        """
        Linearly interpolate between start and end positions based on the distance to travel.
        """
        vector = end - start
        norm = np.linalg.norm(vector)
        if norm == 0:
            return start
        else:
            direction = vector / norm
            return start + direction * distance
    
    def simulate_pose_array(self):
        """
        Simulate and publish object poses based on the loaded trajectories.
        """
        while not rospy.is_shutdown():
            pose_array = StampedObjectPoseArray()
            pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
            
            for i, trajectory in enumerate(self.trajectories):
                if len(trajectory) < 2:
                    continue
                start_index = self.current_indexes[i]
                end_pos = np.array([trajectory[start_index + 1]['x'], trajectory[start_index + 1]['y'], trajectory[start_index + 1]['z']])
                
                # Interpolate the position
                self.current_positions[i] = self.interpolate_position(self.current_positions[i], end_pos, self.step_size)
                
                
                if self.frame_id == 'base_link':
                    # Create the PoseStamped message for transformation
                    pose_stamped = PoseStamped()
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.header.frame_id = 'map'
                    pose_stamped.pose.position.x = self.current_positions[i][0]
                    pose_stamped.pose.position.y = self.current_positions[i][1]
                    pose_stamped.pose.position.z = self.current_positions[i][2]
                    pose_stamped.pose.orientation.w = 1.0  # Assuming no rotation
                    try:
                        if self.tf_buffer.can_transform('base_link', 'map', rospy.Time.now(), rospy.Duration(1.0)):
                            # Transform the pose to the base_link frame
                            transformed_pose = self.tf_buffer.transform(pose_stamped, 'base_link')
                            # Create and append the ObjectPose message
                            pose = ObjectPose()
                            pose.pose.pose.position.x = transformed_pose.pose.position.x
                            pose.pose.pose.position.y = transformed_pose.pose.position.y
                            pose.pose.pose.position.z = transformed_pose.pose.position.z
                            pose.pose.pose.orientation = transformed_pose.pose.orientation
                            pose_array.objects.append(pose)
                        else:
                            rospy.logwarn("Transform from map to base_link is not available.")
                    
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        rospy.logwarn(f"Transform exception: {e}")
                        continue
                else:
                    pose = ObjectPose()
                    pose.pose.pose.position.x = self.current_positions[i][0]
                    pose.pose.pose.position.y = self.current_positions[i][1]
                    pose.pose.pose.position.z = self.current_positions[i][2]
                    pose_array.objects.append(pose)

                # Update the current position index and trajectory
                if np.linalg.norm(self.current_positions[i] - end_pos) < self.step_size:
                    self.current_indexes[i] += 1
                    if self.current_indexes[i] == len(trajectory) - 1:
                        self.current_indexes[i] = 0  # restart trajectory
                        self.current_positions[i] = np.array([trajectory[0]['x'], trajectory[0]['y'], trajectory[0]['z']])
            
            # Publish PoseArray message
            self.pub.publish(pose_array)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        yaml_file_path = '/svea_ws/src/svea_navigation/scripts/util/simulated_trajectories/traj_trial.yaml'   # sidewalk_DKv_30_east_to_west_multiple_pedestrians_rand, sidewalk_DKv_30_east_to_west_one_pedestrian
        publish_frame_id = 'map'            # frame ID where the poses are published. choose between map and base_link
        simulator = ObjectPoseSimulator(yaml_file_path, publish_frame_id)
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass

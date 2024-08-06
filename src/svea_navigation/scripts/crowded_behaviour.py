#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from svea_vision_msgs.msg import StampedObjectPoseArray
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from std_msgs.msg import String


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)

class CrowdedBehaviorMonitor:

    def __init__(self):
        # Initialize ROS node, publishers, subscribers, and services
        rospy.init_node('crowded_behavior_monitor')

        # Parameters defining the cone where the crowdedness is measured 
        self.cluster_safe_distance_x = load_param('~cluster_safe_distance_x', 3.0)
        self.cluster_safe_distance_y = load_param('~cluster_safe_distance_y', 0.8)
        self.min_distance = load_param('~min_distance', 0.3)

        self.angle_max = np.arctan2(self.cluster_safe_distance_y , self.cluster_safe_distance_x)
        self.angle_min = -self.angle_max

        # Transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Obstacles detected by zed camera. (or simulated)
        self.objectposes_subscriber = rospy.Subscriber('/objectposes', StampedObjectPoseArray, self.dynamic_obstacles_callback)

        # Publisher for status messages
        self.status_publisher = rospy.Publisher('/environment_status', String, queue_size=1)

        # Parameters to track in which region of the cone the obstacles are detected.
        self.right_region = False
        self.left_region = False

    def dynamic_obstacles_callback(self, msg):

        # Filter and count obstacles
        obstacle_count = 0
        self.right_region = False
        self.left_region = False

        for obj in msg.objects:
            try:
                # Create PoseStamped message for transformation
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = rospy.Time.now()
                pose_stamped.header.frame_id = 'map'
                pose_stamped.pose = obj.pose.pose

                # Transform obstacle position to base_link frame
                transformed_pose = self.tf_buffer.transform(pose_stamped, 'base_link', rospy.Duration(1.0))
                
                # Extract transformed position
                obstacle_position = transformed_pose.pose.position
                obstacle_point = np.array([obstacle_position.x, obstacle_position.y])
                distance = np.linalg.norm(obstacle_point)
                if distance <= self.cluster_safe_distance_x and self.is_within_angle_range(obstacle_point, self.angle_min, self.angle_max):
                    if distance <= self.min_distance:
                        self.publish_status('emergency_stop')
                        rospy.loginfo(f'Person is too close. Stop the vehicle')
                        break
                    obstacle_count += 1

            except tf2_ros.TransformException as e:
                rospy.logwarn(f"Transform failed: {e}")
        
        if obstacle_count >= 2 and self.right_region is True and self.left_region is True:  # checks if at least 2 people, 1 in the right side, the other on the left side, are in the cautious area.
            self.publish_status('crowded')
            rospy.loginfo(f'Front environment is crowded. Stop the vehicle')
        else:
            self.publish_status('clear')

    def is_within_angle_range(self, point, angle_min, angle_max):
        angle_to_point = np.arctan2(point[1], point[0])
        if 0 < angle_to_point <= angle_max:
            self.right_region = True
            return True
        elif angle_min <= angle_to_point < 0:
            self.left_region = True
            return True
        else:
            return False
    
    def publish_status(self, status):
        self.status_publisher.publish(status)


if __name__ == '__main__':
    try:
        monitor = CrowdedBehaviorMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")

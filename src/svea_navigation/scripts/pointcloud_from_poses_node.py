#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import Empty
import sensor_msgs.point_cloud2 as pc2
from svea_vision_msgs.msg import StampedObjectPoseArray
import numpy as np

class PointCloudPublisher:
    def __init__(self, obstacles_topic, pointcloud_topic, clear_costmap_service, clear_interval):
        """
        A ROS node that publishes PointCloud2 messages.
        
        Parameters:
        - pointcloud_topic: The ROS topic to publish the PointCloud2 data.
        - clear_costmap_service: The ROS service to call for clearing the costmap.
        """
        rospy.init_node('pointcloud_from_poses_node')

        # Initialize the publisher
        self.pointcloud_publisher = rospy.Publisher(pointcloud_topic, PointCloud2, queue_size=1)

        # Service client to clear costmap
        self.clear_costmap_service = rospy.ServiceProxy(clear_costmap_service, Empty)
        
        # Ensure the service is available
        rospy.wait_for_service(clear_costmap_service)
        
        rospy.loginfo("Service %s is available.", clear_costmap_service)

        # Create a subscriber for PoseArray
        rospy.Subscriber(obstacles_topic, StampedObjectPoseArray, self.poses_callback)

        # Time interval for clearing costmap
        self.clear_interval = clear_interval
        self.last_clear_time = rospy.Time.now().to_sec()

    def clear_costmap(self):
        """
        Call the service to clear the costmap.
        """
        try:
            self.clear_costmap_service()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
    
    def poses_callback(self, msg):
        """
        Callback function to convert StampedObjectPoseArray to PointCloud2 and publish it.
        
        Parameters:
        - msg (StampedObjectPoseArray): The message received from the /objectposes topic.
        """
        if self.clear_interval != -1:
            # Check if it's time to clear the costmap
            current_time = rospy.Time.now().to_sec()
            if current_time - self.last_clear_time >= self.clear_interval:
                self.clear_costmap()
                self.last_clear_time = current_time

        # Convert StampedObjectPoseArray to PointCloud2
        points = []
        for obj in msg.objects:
            # Extract x, y from Pose (ignore z for 2D point cloud)
            x = obj.pose.pose.position.x
            y = obj.pose.pose.position.y
            z = 0.0  # Set Z to 0 for 2D points
            points.append((x, y, z))

        # Create PointCloud2 message
        header = msg.header

        cloud = pc2.create_cloud_xyz32(header, points)
        self.pointcloud_publisher.publish(cloud)

if __name__ == '__main__':
    try:
        pointcloud_topic = '/obstacle_pointcloud'
        clear_costmap_service = '/move_base/clear_costmaps'
        obstacles_topis = '/objectposes'
        clear_interval = 2     # time in seconds after which the local costmap gets cleaned. -1 = do not clear.
        
        publisher = PointCloudPublisher(obstacles_topis, pointcloud_topic, clear_costmap_service, clear_interval)
        rospy.spin() 
        
    except rospy.ROSInterruptException:
        pass

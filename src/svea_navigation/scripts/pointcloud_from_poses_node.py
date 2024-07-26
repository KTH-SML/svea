#! /usr/bin/env python3

import rospy
from svea_vision_msgs.msg import StampedObjectPoseArray, ObjectPose
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2                  # module to work with PointCloud2 data type
import std_msgs.msg
import numpy as np

def poses_callback(msg):
    # Convert StampedObjectPoseArray to PointCloud2
    points = []
    for obj in msg.objects:
        # Extract x, y from Pose (ignore z for 2D point cloud)
        x = obj.pose.pose.position.x
        y = obj.pose.pose.position.y
        z = 0.0  # Set Z to 0 for 2D points
        points.append((x, y, z))

    # Create PointCloud2 message
    header = std_msgs.msg.Header()
    header = msg.header 

    cloud = point_cloud2.create_cloud_xyz32(header, points)

    # Publish the PointCloud2 message
    pub.publish(cloud)

if __name__ == '__main__':
    rospy.init_node('pointcloud_from_poses_node')

    # Create a subscriber for PoseArray
    rospy.Subscriber('/objectposes', StampedObjectPoseArray, poses_callback)
    # Create a publisher for PointCloud2
    pub = rospy.Publisher('/obstacle_pointcloud', PointCloud2, queue_size=10)

    rospy.spin()

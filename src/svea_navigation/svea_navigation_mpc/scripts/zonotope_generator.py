#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from svea_vision_msgs.msg import StampedObjectPoseArray
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32

def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)

class ZonotopeGenerator:
    """
    Description: ROS node that converts StampedObjectPoseArray messages (obstacles detected by ZED camera) into
    zonotopes (polygons) that will be then provided directly as obstacles to the TEB local planner through the
    CostmapToPolygonsDBSMCCH costmap converter plugin.
    
    Purpose: The node aims at generating the zonotopes that eventually will come from pedestrian 
    reachability analysis. It is supposed for the moment that each obstacle is enlarged with the same zonotope,
    facing the same direction (x direction of map frame in particular) for the sake of simplicity and 
    easier TEB tuning.
    Eventually, when the reachability analysis will be incorporated, precise knowledge of the zonotope's dimensions
    and heading for each obstacle will be required.

    Status: Currently, this feature is primarily used to inflate detected objects rather than as a true
    zonotope generator. While it has shown decent performance on the SVEA, its effectiveness is limited by
    the reliability of the object pose estimates.
    """
    
    def __init__(self):
        rospy.init_node('zonotope_generator_node')

        # Load parameters from the ROS parameter server
        self.obstacles_topic = load_param('~obstacles_topic', '/objectposes')
        self.obstacle_pub_topic = load_param('~obstacle_pub_topic', '/move_base/TebLocalPlannerROS/obstacles')
        self.zonotope_width = load_param('~zonotope_width', 0.7)
        self.zonotope_height = load_param('~zonotope_height', 0.3)
        
        # Initialize the publisher
        self.obstacle_publisher = rospy.Publisher(self.obstacle_pub_topic, ObstacleArrayMsg, queue_size=1)
        
        # Create subscribers
        rospy.Subscriber(self.obstacles_topic, StampedObjectPoseArray, self.poses_callback)

    def generate_zonotopes(self, points_with_ids):
        """
        Inflate the obstacles around the given points with a rectangular inflation area and publish rectangles.
        """
        # Create an ObstacleArrayMsg to publish rectangles
        obstacle_array_msg = ObstacleArrayMsg()
        obstacle_array_msg.header = Header()
        obstacle_array_msg.header.stamp = rospy.Time.now()
        obstacle_array_msg.header.frame_id = 'map'  

        for obj_id, px, py in points_with_ids:
            # Create a rectangular obstacle message
            obstacle_msg = ObstacleMsg()
            obstacle_msg.header = Header()
            obstacle_msg.header.stamp = rospy.Time.now()
            obstacle_msg.header.frame_id = 'map'
            obstacle_msg.id = obj_id

            # Define the corners of the rectangle
            corner1 = (px - self.zonotope_width / 2, py - self.zonotope_height / 2)
            corner2 = (px + self.zonotope_width / 2, py - self.zonotope_height / 2)
            corner3 = (px + self.zonotope_width / 2, py + self.zonotope_height / 2)
            corner4 = (px - self.zonotope_width / 2, py + self.zonotope_height / 2)

            obstacle_msg.polygon.points = [
                Point32(corner1[0], corner1[1], 0),
                Point32(corner2[0], corner2[1], 0),
                Point32(corner3[0], corner3[1], 0),
                Point32(corner4[0], corner4[1], 0),
            ]

            # Add obstacle to the array message
            obstacle_array_msg.obstacles.append(obstacle_msg)

        # Publish the obstacle array
        self.obstacle_publisher.publish(obstacle_array_msg)

    def poses_callback(self, msg):
        """
        Callback for PoseArray messages.
        """
        # Convert StampedObjectPoseArray to points with IDs
        points_with_ids = []
        for obj in msg.objects:
            obj_id = obj.object.id
            x = obj.pose.pose.position.x
            y = obj.pose.pose.position.y
            z = 0.0  # 2D points
            points_with_ids.append((obj_id, x, y))
  
        self.generate_zonotopes(points_with_ids)

if __name__ == '__main__':
    try:
        ZonotopeGenerator()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

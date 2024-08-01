#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from std_srvs.srv import Empty
from svea_vision_msgs.msg import StampedObjectPoseArray
from nav_msgs.msg import OccupancyGrid
import numpy as np


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


class DynamicObstaclesInflator:
    def __init__(self, obstacles_topic, local_costmap_topic, clear_costmap_service, clear_interval):
        rospy.init_node('dynamic_obstacles_inflator_node')

        # Parameters
        self.inflation_width = load_param('~inflation_width', 2.0)
        self.inflation_height = load_param('~inflation_height', 1)
        self.cost_scaling_factor = load_param('~cost_scaling_factor', 1.0)

        # Initialize the publisher
        self.costmap_publisher = rospy.Publisher(local_costmap_topic, OccupancyGrid, queue_size=1)

        # Service client to clear costmap
        self.clear_costmap_service = rospy.ServiceProxy(clear_costmap_service, Empty)
        
        # Ensure the service is available
        rospy.wait_for_service(clear_costmap_service)
        rospy.loginfo("Service %s is available.", clear_costmap_service)

        # Create subscribers
        rospy.Subscriber(obstacles_topic, StampedObjectPoseArray, self.poses_callback)
        rospy.Subscriber(local_costmap_topic, OccupancyGrid, self.costmap_callback)

        # Time interval for clearing costmap
        self.clear_interval = clear_interval
        self.last_clear_time = rospy.Time.now().to_sec()

        self.costmap = None
        self.points = []

    def clear_costmap(self):
        """
        Call the service to clear the costmap.
        """
        try:
            self.clear_costmap_service()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def inflate_points(self, points):
        """
        Inflate the costmap around the given points with a rectangular inflation area.

        :param points: List of (x, y) coordinates to be inflated.
        """
        if self.costmap is None:
            rospy.logwarn("Costmap is not initialized.")
            return
        # Get the resolution and dimensions of the costmap
        resolution = self.costmap.info.resolution
        width = self.costmap.info.width
        height = self.costmap.info.height
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y

        # Create a copy of the original costmap data to apply the inflation
        inflated_costmap = np.array(self.costmap.data).reshape((height, width))

        # Inflation dimensions in grid cells
        inflation_width_cells = int(self.inflation_width / resolution)
        inflation_height_cells = int(self.inflation_height / resolution)

        for (px, py, _) in points:
            # Convert point coordinates to pixel coordinates
            wx = px - origin_x
            wy = py - origin_y
            px_pix = int(wx / resolution)
            py_pix = int(wy / resolution)

            # Define the rectangular area for inflation
            x_min = max(px_pix - inflation_width_cells // 2, 0)
            x_max = min(px_pix + inflation_width_cells // 2 + 1, width)
            y_min = max(py_pix - inflation_height_cells // 2, 0)
            y_max = min(py_pix + inflation_height_cells // 2 + 1, height)

            # Update the costmap within the defined rectangular area
            for x in range(x_min, x_max):
                for y in range(y_min, y_max):
                    # Calculate distance from the obstacle center
                    distance_x = abs(x - px_pix) * resolution
                    distance_y = abs(y - py_pix) * resolution
                    distance = max(distance_x, distance_y)
                    
                    # Calculate the cost based on the distance
                    if distance <= max(self.inflation_width / 2, self.inflation_height / 2):
                        # Start with maximum cost and linearly decay
                        cost = max(0, 100 - int(self.cost_scaling_factor * distance))
                        # Update the costmap with the calculated cost
                        inflated_costmap[y, x] = max(inflated_costmap[y, x], cost)

        # Convert the inflated costmap back to the format needed for publishing
        self.costmap.data = inflated_costmap.flatten().tolist()
        self.costmap_publisher.publish(self.costmap)



    def poses_callback(self, msg):
        """
        Callback for PoseArray messages.
        """
        # Check if it's time to clear the costmap
        current_time = rospy.Time.now().to_sec()
        if current_time - self.last_clear_time >= self.clear_interval:
            self.clear_costmap()
            self.last_clear_time = current_time

        # Convert StampedObjectPoseArray to points
        self.points = []
        for obj in msg.objects:
            # Extract x, y from Pose (ignore z for 2D points)
            x = obj.pose.pose.position.x
            y = obj.pose.pose.position.y
            z = 0.0  # Set Z to 0 for 2D points
            self.points.append((x, y, z))


    def costmap_callback(self, msg):
        """
        Callback for Costmap2D messages.
        """
        self.costmap = msg
        
        # Inflate the points on the costmap
        self.inflate_points(self.points)

if __name__ == '__main__':
    try:
        local_costmap_topic = '/move_base/local_costmap/costmap'
        clear_costmap_service = '/move_base/clear_costmaps'
        obstacles_topic = '/objectposes'
        clear_interval = 10000000  # Time in seconds after which the local costmap gets cleaned if any dynamic obstacle is detected
        
        publisher = DynamicObstaclesInflator(obstacles_topic, local_costmap_topic, clear_costmap_service, clear_interval)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

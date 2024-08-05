import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import numpy as np

class CrowdedBehaviorMonitor:
    def __init__(self):
        # Initialize ROS node, publishers, subscribers, and services
        rospy.init_node('crowded_behavior_monitor')
        self.dynamic_obstacles_subscriber = rospy.Subscriber('/move_base/TebLocalPlannerROS/obstacles', MarkerArray, self.dynamic_obstacles_callback)
        self.transform_listener = TransformListener()  # To get the transformation between frames
        self.is_crowded = False

    def dynamic_obstacles_callback(self, msg):
        try:
            # Get the current transformation from base_link to map
            self.transform_listener.waitForTransform('base_link', 'map', rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.transform_listener.lookupTransform('base_link', 'map', rospy.Time(0))
            heading = euler_from_quaternion(rot)[2]  # Get the yaw (heading) angle

            # Define the angular range (±π/3) and radius (1 meter)
            angle_min = -np.pi / 3
            angle_max = np.pi / 3
            radius = 1.0

            # Filter and count obstacles
            obstacle_count = 0
            for marker in msg.markers:
                # Transform obstacle position to base_link frame
                obstacle_point = self.transform_point_to_base_link(marker.pose.position, trans)
                distance = np.linalg.norm(obstacle_point)

                if distance <= radius and self.is_within_angle_range(obstacle_point, heading, angle_min, angle_max):
                    obstacle_count += 1

            # Set is_crowded based on obstacle count
            self.is_crowded = obstacle_count > 2

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logwarn("Transform lookup failed")

    def transform_point_to_base_link(self, point, transform):
        # Apply the transform to convert the obstacle point to base_link frame
        point_transformed = np.array([point.x - transform.translation.x,
                                      point.y - transform.translation.y])
        return point_transformed

    def is_within_angle_range(self, point, heading, angle_min, angle_max):
        angle_to_point = np.arctan2(point[1], point[0])
        angle_to_point = (angle_to_point + 2 * np.pi) % (2 * np.pi)  # Normalize to [0, 2π)
        heading = (heading + 2 * np.pi) % (2 * np.pi)
        angle_min = (heading + angle_min + 2 * np.pi) % (2 * np.pi)
        angle_max = (heading + angle_max + 2 * np.pi) % (2 * np.pi)
        return angle_min <= angle_to_point <= angle_max

    def stop_vehicle(self):
        # Stop the vehicle by setting control commands to zero
        self.publish_control(0, 0)

    def publish_control(self, steering, velocity):
        # Publish control commands
        pass

    def spin(self):
        while not rospy.is_shutdown():
            # Main loop logic
            if self.is_crowded:
                rospy.loginfo("Sidewalk is crowded. Stopping vehicle...")
                self.stop_vehicle()
                # Implement waiting and clearing costmaps as needed
            else:
                rospy.loginfo("Sidewalk is clear. Proceeding with navigation...")
                # Implement normal control and navigation
            rospy.sleep(0.1)  # Adjust the loop rate as needed

if __name__ == '__main__':
    monitor = CrowdedBehaviorMonitor()
    monitor.spin()

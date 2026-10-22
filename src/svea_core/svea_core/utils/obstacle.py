from .. import rosonic as rx
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
#from svea_msgs.msg import Circle, Circle_array
import tf2_ros


from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import rx

qos_subber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable
    history=QoSHistoryPolicy.KEEP_LAST,  # Keep the last N messages
    durability=QoSDurabilityPolicy.VOLATILE,  # Volatile
    depth=10,  # Size of the queue
)

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class LaserScanToCircle(rx.Field):

    radius = rx.Parameter(0.5) # Default radius: 0.5
    min_elements = rx.Parameter( 5) # Default min_elements: 5
    sparsity = rx.Parameter( False)

    marker_pub = rx.Publisher(MarkerArray, "vis_circles", qos_pubber)
    #obstacles_publisher = rx.Publisher(Circle_array, "/obstacles",qos_pubber)

    @rx.Subscriber(LaserScan, "/scan" , qos_subber)
    def scan_callback(self, scan_msg):
        points = self.convert_scan_to_points(scan_msg)

        circles = self.calculate_circles(points)

        if self.sparsity:
            circles = self.enforce_sparsity(circles)

        transformed_circles = self.transform_circles(circles, "base_link", "map")

        self.publish_circles(transformed_circles)  # Visualization
        # self.publish_obstacles(transformed_circles)  # For the MPC

    def convert_scan_to_points(self, scan_msg):
        """Convert LaserScan message to (x, y) points."""
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
        ranges = np.array(scan_msg.ranges)

        valid_mask = np.isfinite(ranges) & (ranges > 0)
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        x = ranges * np.cos(angles) + 0.3
        y = ranges * np.sin(angles)

        return np.stack((x, y), axis=-1)

    def calculate_circles(self, points):
        """Group points into circles, assuming points are in order left to right."""
        if len(points) == 0:
            return []

        circles = []
        current_center = points[0]
        current_count = 1

        for i in range(1, len(points)):
            distance = np.linalg.norm(points[i] - current_center)
            if distance <= self.radius:
                current_count += 1
            else:
                # Add the current circle if it meets the minimum elements condition
                if current_count >= self.min_elements:
                    circles.append((current_center[0], current_center[1]))
                # Start a new circle
                current_center = points[i]
                current_count = 1

        # Add the last circle if it meets the condition
        if current_count >= self.min_elements:
            circles.append((current_center[0], current_center[1]))

        return circles


    def transform_circles(self, circles, from_frame, to_frame):
        try:
            transform = self.tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))

            translation = np.array([transform.transform.translation.x,
                                    transform.transform.translation.y])

            q = transform.transform.rotation
            cos_theta = 1 - 2 * (q.y ** 2 + q.z ** 2)
            sin_theta = 2 * (q.w * q.z + q.x * q.y)
            rotation_matrix_2d = np.array([[cos_theta, -sin_theta],
                                           [sin_theta, cos_theta]])

            circles_np = np.array(circles)
            if not len(circles) == 0:
                transformed_circles = circles_np @ rotation_matrix_2d.T + translation
            else:
                transformed_circles = circles_np
            return transformed_circles.tolist()

        except tf2_ros.TransformException as ex:
            rospy.logerr(f"Transform error: {ex}")
            return circles  # Return unchanged circles on failure

    def publish_circles(self, circles):
        """Publish the circles as markers for visualization."""
        marker_array = MarkerArray()
        for i, (center_x, center_y) in enumerate(circles):
            marker = Marker()
            marker.header.frame_id = "map"  # Adjust frame ID as needed
            marker.header.stamp = rospy.Time.now()
            marker.ns = "vis_circles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = 0  # Flat on the ground
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.radius * 2  # Diameter in x
            marker.scale.y = self.radius * 2  # Diameter in y
            marker.scale.z = 0.01  # Small height for visualization
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5  # Semi-transparent
            marker.lifetime = rospy.Duration(0.1)  # Short lifetime to refresh with each scan

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def enforce_sparsity(self, circles, epsilon=0.001):
        """Filter circles to enforce sparsity by removing overlapping circles."""
        sparse_circles = []
        for circle in circles:
            if all(np.linalg.norm(np.array(circle) - np.array(existing)) > self.radius * 2 + epsilon
                   for existing in sparse_circles):
                sparse_circles.append(circle)
        return sparse_circles

from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import MapMetaData, OccupancyGrid
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import rospy


class ROSInterface(object):
    """Map ROSInterface used to handle all ROS related functions
    """

    @staticmethod
    def get_occupancy_grid_parameters():
        """Creates a dictionary representation of the parameters that will be
        used for the occupancy grid.

        :return: dictionary of ROS parameters
        :rtype: dictionary
        """
        parameter_names = [
            'og_width', 'og_height', 'og_resolution',
            'og_origin_x', 'og_origin_y', 'obstacles'
        ]

        parameters = {
            parameter_name: rospy.get_param('~{}'.format(parameter_name))
            for parameter_name in parameter_names
        }

        occupancy_grid_parameters = {
            'width': parameters['og_width'],
            'height': parameters['og_height'],
            'origin': (parameters['og_origin_x'], parameters['og_origin_y']),
            'resolution': parameters['og_resolution'],
            'obstacles': parameters['obstacles']
        }

        return occupancy_grid_parameters

    @staticmethod
    def publish(occupancy_grid):
        """Publishes all topics related to Map: occupancy grid & obstacles

        :param occupancy_grid: ocuppancy grid representation of the map
        :type occupancy_grid: OccupancyGrid object
        """
        ROSInterface._publish_occupancy_grid(occupancy_grid)
        ROSInterface._publish_obstacles(occupancy_grid.obstacles)

    @staticmethod
    def _publish_occupancy_grid(occupancy_grid):
        """Publishes and latches a OccupancyGrid ROS message, given a occupancy
        grid object as the input.

        :param occupancy_grid: ocuppancy grid representation of the map
        :type occupancy_grid: OccupancyGrid object
        """
        map_publisher = rospy.Publisher(
            '~map', OccupancyGrid, queue_size=1, latch=True
        )

        x0, y0 = occupancy_grid.origin

        map_origin = Pose(position=Point(x=x0, y=y0, z=0))

        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = rospy.get_param('~path_frame')

        map_msg.info = MapMetaData(
            resolution=occupancy_grid.resolution,
            width=occupancy_grid.width,
            height=occupancy_grid.height,
            origin=map_origin
        )

        map_msg.data = list(occupancy_grid.grid_data.T.flatten())

        map_publisher.publish(map_msg)

    @staticmethod
    def _publish_obstacles(obstacles):
        """Publishes and latches a MarkerArray ROS message,
        given a list of obstacles (each one defined by its edges).

        :param obstacles: List of obstacles (each one with obstacle edges)
        :type obstacles: list
        """
        obstacles_publisher = rospy.Publisher(
            '~obstacles', MarkerArray, queue_size=1, latch=True
        )

        markers = [
            ROSInterface._create_marker_from_obstacle_edges(obstacle_edges, id)
            for id, obstacle_edges in enumerate(obstacles)
        ]

        obstacles_msg = MarkerArray(markers=markers)

        obstacles_publisher.publish(obstacles_msg)

    @staticmethod
    def _create_marker_from_obstacle_edges(obstacle_edges, id):
        """Creates a Marker ROS message, given the edges of an obstacle as the
        input and an id to keep track of each Marker internally.

        :param obstacle_edges: List of edges with coordinates [x, y]
        :type obstacle_edges: list
        :param id: unique identifier for the Marker
        :type id: int
        :return: polygon-like object using line strips
        :rtype: Marker ROS message
        """
        marker_msg = Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = rospy.get_param('~path_frame')

        marker_msg.type = Marker.LINE_STRIP
        marker_msg.action = Marker.ADD
        marker_msg.points = [Point(x=edge[0], y=edge[1])
                             for edge in obstacle_edges + [obstacle_edges[0]]]

        marker_msg.id = id
        marker_msg.color = ColorRGBA(r=1, g=0, b=0, a=1)
        marker_msg.scale.x = 0.05
        marker_msg.pose.orientation.w = 1.0

        return marker_msg

import numpy as np
from scipy.spatial import Delaunay


class OccupancyGrid(object):
    """Occupancy Grid representation of a map. It follows the conventions
    of the ROS message OccupancyGrid in terms of parameters

    :param width: map width, in cells
    :type width: int
    :param height: map height, in cells
    :type height: int
    :param resolution: map resolution in m/cells, defaults to 1
    :type resolution: int, optional
    :param obstacles: List of obstacles (each one with obstacle edges)
    :type obstacles: list
    """

    EMPTY = 0
    FULL = 100

    def __init__(self, width, height, origin=(0, 0), resolution=1, obstacles=[]):
        ## The map resolution [m/cell]
        self.resolution = resolution
        ## Map width [cells]
        self.width = width
        ## Map height [cells]
        self.height = height
        ## The origin of the map [m, m]
        self.origin = origin
        ## Map data, occupancy probabilities in the range [0, 100]. Unknown is -1
        self.grid_data = np.zeros((self.width, self.height))
        ## Obstacles represented by edges of a polyhedron
        self.obstacles = obstacles

        self._add_obstacles_to_map()

    @property
    def total_width_m(self):
        """Gets the total width of the map, in m

        :return: total width of the map, in m
        :rtype: float
        """
        return self.width * self.resolution

    @property
    def total_height_m(self):
        """Gets the total height of the map, in m

        :return: total height of the map, in m
        :rtype: float
        """
        return self.height * self.resolution

    def is_node_within_map(self, node):
        """Checks if the node being passed as an argument is within the map

        :param node: Tuple representing a cell with coordinates (x, y) or (x, y, yaw)
        :type node: tuple
        :return: `True` if the node is within the map, `False` otherwise
        :rtype: bool
        """
        node = self._normalize_node_to_map_origin(node)

        check_x = node[0] >= 0 and node[0] < self.width * self.resolution
        check_y = node[1] >= 0 and node[1] < self.height * self.resolution

        return check_x and check_y

    def get_closest_cell(self, node):
        """Given an input node with continuous values, it interpolates and
        gets the closest cell available in the map (discrete cell)

        :param node: Tuple representing a cell with coordinates (x, y) or (x, y, yaw)
        :type node: tuple
        :return: closest cell (x, y) (or (x, y, yaw)) available in the map
        :rtype: tuple
        """
        node = self._normalize_node_to_map_origin(node)

        closest_x = int(node[0] / self.resolution + 0.5) * self.resolution
        closest_y = int(node[1] / self.resolution + 0.5) * self.resolution

        closest_node = (closest_x, closest_y)

        closest_node = self._restore_node_from_map_origin(closest_node)
        # Preserve other indices (e.g. orientation)
        return closest_node + node[2:]

    def _add_obstacles_to_map(self):
        """Given a list of obstacles (each one defined by its edges), it fills
        the grid cells that are contained within those obstacles.
        """
        for obstacle_edges in self.obstacles:
            self._add_obstacle_from_edges(obstacle_edges)

    def _add_obstacle_from_edges(self, obstacle_edges):
        """Given the edges of an obstacle, it fills the grid cells that are
        contained within the obstacle.

        :param obstacle_edges: List of edges with coordinates [x, y]
        :type obstacle_edges: list
        """
        normalized_obstacle_edges = [
            self._normalize_node_to_map_origin(obstacle_edge)
            for obstacle_edge in obstacle_edges
        ]

        hull = Delaunay(normalized_obstacle_edges)

        for x_index in range(self.width):
            for y_index in range(self.height):
                points = np.array([
                    [(x_index + 0.1 * offset_x) * self.resolution,
                     (y_index + 0.1 * offset_y) * self.resolution]
                    for offset_x in range(1, 10)
                    for offset_y in range(1, 10)
                ])

                if any(hull.find_simplex(points) >= 0):
                    self.grid_data[x_index, y_index] = self.FULL

    def _normalize_node_to_map_origin(self, node):
        """Translates node from global coordinates to coordinates relative to map origin

        :param node: Tuple representing a cell with coordinates (x, y) or (x, y, yaw).
                      List representing a cell with coordinates [x, y]
        :type node: tuple or list
        :return: normalized tuple (x, y) (or (x, y, yaw)) with coordinates relative to the map.
        :rtype: tuple
        """
        node_x = node[0] - self.origin[0]
        node_y = node[1] - self.origin[1]

        return (node_x, node_y) + tuple(node[2:])

    def _restore_node_from_map_origin(self, node):
        """Translate node back from coordinates relative to map origin to global coordinates

        :param node: Normalized tuple (x, y) (or (x, y, yaw)) with coordinates relative to the map.
        :type node: tuple
        :return: Tuple representing a cell with coordinates (x, y) or (x, y, yaw).
        :rtype: tuple
        """
        node_x = node[0] + self.origin[0]
        node_y = node[1] + self.origin[1]

        return (node_x, node_y) + tuple(node[2:])

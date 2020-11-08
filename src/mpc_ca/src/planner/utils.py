import numpy as np
from scipy.interpolate import splprep, splev


def interpolate_path(path, count_interpolated_points):
    """Interpolates a given path and a number of desired interpolated points

    :param path: path, where each pose is of the format (x, y, yaw)
    :type path: list
    :param count_interpolated_points: number of desired interpolated points
    :type count_interpolated_points: int
    :return: interpolated path, where each pose is ofthe format (x, y, yaw)
    :rtype: np.array
    """
    path = np.array(path)

    path_x = path[:, 0]
    path_y = path[:, 1]
    path_angle = path[:, 2]

    tck, _ = splprep([path_x, path_y, path_angle])

    path_x, path_y, path_angle = splev(
        np.linspace(0, 1, count_interpolated_points), tck
    )

    return np.column_stack((path_x, path_y, path_angle))


def calculate_distance(first_node, second_node):
    """Calculates the euclidean distance between two nodes

    :param first_node: node, in the format (x, y) or (x, y, yaw)
    :type first_node: tuple
    :param second_node: node, in the format (x, y) or (x, y, yaw)
    :type second_node: tuple
    :return: distance between the two nodes
    :rtype: float
    """
    first_node = np.array(first_node[0:2])
    second_node = np.array(second_node[0:2])

    return np.linalg.norm(second_node - first_node)


def are_nodes_close_enough(first_node, second_node, distance_tolerance=None,
                            angle_tolerance=None):
    """Determines if two nodes are close enough, given a distance and an
    angle tolerance

    :param first_node: node, in the format (x, y) or (x, y, yaw)
    :type first_node: tuple
    :param second_node: node, in the format (x, y) or (x, y, yaw)
    :type second_node: tuple
    :param distance_tolerance: minimum distance error to consider
                               both nodes close enough, defaults to None
    :type distance_tolerance: float, optional
    :param angle_tolerance: minimum angle error to consider
                            both nodes close enough, defaults to None
    :type angle_tolerance: float, optional
    :return: `True` if the nodes are close enough, `False` otherwise
    :rtype: bool
    """
    if first_node is None or second_node is None:
        return False

    if distance_tolerance is None and angle_tolerance is None:
        return False

    close_enough = True

    if distance_tolerance is not None:
        close_enough = (
            close_enough
            and calculate_distance(first_node, second_node) <= distance_tolerance
        )

    if angle_tolerance is not None:
        close_enough = (
            close_enough
            and abs(second_node[2] - first_node[2]) <= angle_tolerance
        )

    return close_enough

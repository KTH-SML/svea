from planner.mpc import NonLinearMPC
from planner.ros_interface import Logger as logging


class Planner(object):
    """Planner wrapper to handle the different planner algorithms available.

    :param occupancy_grid: ocuppancy grid representation of the map
    :type occupancy_grid: OccupancyGrid object
    :param planner_name: name of the planner algorithm
    :type planner_name: str
    :param planner_parameters: ROS parameters for the planner algorithm
    :type planner_parameters: dict
    :raises RuntimeError: An invalid planner name was provided
    """

    available_planners = ['mpc']
    available_initial_guess_planners = []

    def __init__(self, occupancy_grid, planner_name, planner_parameters):
        if planner_name not in self.available_planners:
            logging.error('"{}" is not a valid planner!'.format(planner_name))
            raise RuntimeError

        self.occupancy_grid = occupancy_grid

        self.planner = NonLinearMPC(planner_parameters)
        self.planner.setup_optimization_problem(occupancy_grid.obstacles)

    def compute_path(self, initial_state, goal_state):
        """Uses the planner to compute the path

        :param initial_state: initial pose [x, y, yaw]
        :type initial_state: list
        :param goal_state: goal pose [x, y, yaw]
        :type goal_state: list
        :return: path, where each pose is of the format [x, y, yaw]
        :rtype: list of tuples or numpy array
        """
        planner_kwargs = {
            'initial_state': initial_state,
            'goal_state': goal_state
        }
        return self.planner.compute_path(**planner_kwargs)


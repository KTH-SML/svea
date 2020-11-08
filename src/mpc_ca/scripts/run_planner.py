#! /usr/bin/env python
"""
Path planning with non-linear MPC
"""
from map.occupancy_grid import OccupancyGrid
from map.ros_interface import ROSInterface as MapROSInterface
from planner.main_planner import Planner
from planner.ros_interface import ROSInterface


if __name__ == '__main__':
    ros_interface = ROSInterface()

    occupancy_grid_parameters = MapROSInterface.get_occupancy_grid_parameters()

    occupancy_grid = OccupancyGrid(**occupancy_grid_parameters)

    MapROSInterface.publish(occupancy_grid)

    planner_name = ros_interface.planner_algorithm
    planner_parameters = ros_interface.get_planner_parameters()

    planner = Planner(occupancy_grid, planner_name, planner_parameters)

    while not ros_interface.is_shutdown():
        ros_interface.sleep()

        if not ros_interface.has_endpoint_changed:
            continue

        initial_state = ros_interface.initial_state
        goal_state = ros_interface.goal_state
        path = planner.compute_path(initial_state, goal_state)
        ros_interface.publish_path(path)

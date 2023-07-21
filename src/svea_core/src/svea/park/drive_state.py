#! /usr/bin/env python3

import numpy as np
import rospy
import math

from svea.park.path_smoothing import smooth_path

from std_msgs.msg import Float64MultiArray


class DriveState():

    def __init__(self, controller, goal_threshold_pos=0.25, goal_threshold_yaw=10*math.pi/180):
        self.controller = controller
        self.goal_threshold_pos = goal_threshold_pos
        self.goal_threshold_yaw = goal_threshold_yaw
        self.trajectory = [[1434, 1434]]
        self.path_finished = False

        rospy.loginfo("Drive state initialized, using: " + self.controller.__class__.__name__)

        
    def run(self, vehicle: object)-> object:
        print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++", vehicle)
        if self.path_is_new(vehicle.path): # If the path has changed
            self.update_traj(vehicle)

        self.control_step(vehicle) 

        vehicle.data_handler.update_target(self.controller.target) # Update target plot

        if self.path_is_finished(vehicle.state): 
            rospy.loginfo("Path finished")
            vehicle.path_finished = True
            self.clear_trajectory(vehicle = vehicle)         # Clear old trajectory
        return vehicle


    def clear_trajectory(self, vehicle: object) -> None:
        self.controller.target_idx = 0  # Reset controller index
        self.controller.first_time = True

        # Reset controller target
        # self.controller.target = [0,0]

        #self.trajectory = [[0,0]]

        #self.controller.traj_x = np.asarray([0])
        #self.controller.traj_y = np.asarray([0])
        #self.controller.traj_yaw = np.asarray([0])

        vehicle.data_handler.update_traj(self.controller.traj_x, self.controller.traj_y) # Update target plot
        

    def control_step(self, vehicle: object) -> None: # Compute and send control
        steering, velocity = self.controller.compute_control(vehicle.state)
        velocity = velocity * vehicle.speed_guard.multiplier
        vehicle.speed_guard.publish_steering_angle(steering)
        vehicle.send_control(steering, velocity) 
        
            
    def update_traj(self, vehicle: object) -> None:
        self.trajectory = np.array(smooth_path(vehicle.path)) # Generates trajectory and then smoothens it
        assert len(self.trajectory[0]) == len(self.trajectory[1])
        
        self.controller.traj_x = np.asarray(self.trajectory)[:, 0]
        self.controller.traj_y = np.asarray(self.trajectory)[:, 1]
        self.controller.traj_yaw = np.asarray(self.trajectory)[:, 2]

        vehicle.data_handler.update_traj(self.controller.traj_x,
                                                         self.controller.traj_y)
        rospy.loginfo("Trajectory generated (drive state)")


    def path_is_new(self, new_path):
        return self.trajectory[0][0] != new_path[0][0] or self.trajectory[0][1] != new_path[0][1]


    def path_is_finished(self, state):
        end_point = self.trajectory[-1]
        pos_err = np.linalg.norm(np.array([state.x, state.y]) - np.array([end_point[0], end_point[1]]))
        yaw_err = np.abs(state.yaw - end_point[2])
        return pos_err < self.goal_threshold_pos and yaw_err < self.goal_threshold_yaw
    
    def update_goal(self):
        self.curr += 1
        self.curr %= len(self.POINTS)
        self.goal = self.POINTS[self.curr]
        self.svea.controller.is_finished = False

    def compute_traj(self, state):
        xs = np.linspace(state.x, self.goal[0], self.TRAJ_LEN)
        ys = np.linspace(state.y, self.goal[1], self.TRAJ_LEN)
        return xs, ys
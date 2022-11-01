#!/usr/bin/env python

import math
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray
from svea.simulators.viz_utils import publish_pose_array

# SVEA DIMENSIONS
LENGTH = 0.586  # [m]
BACKTOWHEEL = 0.16  # [m]
FRONTTOWHEEL = LENGTH - BACKTOWHEEL  # [m]


def rotate2D(xy, radians):
    x, y = xy
    rot = np.matrix([[np.cos(radians), np.sin(radians)],
                     [-np.sin(radians), np.cos(radians)]])
    return np.dot(rot, [x, y]).tolist()[0]


def compute_offset(spacing, heading):
    # compute distance between two neighbor vehicles' rear axles (where state is)
    dist = (BACKTOWHEEL + spacing + FRONTTOWHEEL)
    xy = [dist, 0]
    rotated_xy = rotate2D(xy, heading)
    return np.array(rotated_xy + [0.0, 0.0])


def compute_positions_from_spacings(last_vehicle_pt, spacings):
    # base placements on last vehicle to maximize runway length
    fixed_pt = np.array(last_vehicle_pt)
    heading = fixed_pt[2]

    follower_pts = np.zeros((len(spacings), 4))
    follower_pts[-1, :] = fixed_pt
    total_offset = 0
    for reverse_i, spacing in enumerate(reversed(spacings[1:])):
        offset = compute_offset(spacing, -heading)

        i = len(spacings) - 2 - reverse_i
        follower_pts[i, :] = fixed_pt + total_offset + offset
        total_offset += offset

    offset = compute_offset(spacings[0], -heading)
    leader_pt = fixed_pt + total_offset + offset

    return leader_pt, follower_pts


def collect_platoon_pts(leader_pt, follower_pts):
    xs = [leader_pt[0]] + [follower_eq_pt[0] for follower_eq_pt in follower_pts]
    ys = [leader_pt[1]] + [follower_eq_pt[1] for follower_eq_pt in follower_pts]
    yaws = [leader_pt[2]] \
           + [follower_eq_pt[2] for follower_eq_pt in follower_pts]
    return xs, ys, yaws


def goto_eq_positions(leader, leader_eq_pt, followers, follower_eq_pts):

    init_pts_publisher = rospy.Publisher("/c_ovrv_init_pts",
                                         PoseArray, queue_size=1)
    eq_xs, eq_ys, eq_yaws = collect_platoon_pts(leader_eq_pt, follower_eq_pts)
    publish_pose_array(init_pts_publisher, eq_xs, eq_ys, eq_yaws)

    while not rospy.is_shutdown():
        publish_pose_array(init_pts_publisher, eq_xs, eq_ys, eq_yaws)
        steering, velocity = leader.goto_pt(leader_eq_pt)
        if velocity == 0.0:
            break
        else:
            leader.send_control(steering, velocity)
            leader.data_handler.pub_car_poly()
    rospy.loginfo("- Leader Ready")

    for i, follower in enumerate(followers):
        while not rospy.is_shutdown():
            publish_pose_array(init_pts_publisher, eq_xs, eq_ys, eq_yaws)
            steering, velocity = follower.goto_pt(follower_eq_pts[i])
            if velocity == 0.0:
                break
            else:
                follower.send_control(steering, velocity)
                follower.data_handler.pub_car_poly()
        rospy.loginfo("- Follower {0} Ready".format(i))


def reached_steady_state(steady_vel, leader, followers):
    leader_is_steady = abs(leader.state.v - steady_vel)/steady_vel < 0.1 # 10%
    followers_are_steady = [abs(follower.state.v-steady_vel)/steady_vel < 0.1
                            for follower in followers]
    return leader_is_steady and all(followers_are_steady)


def compute_spacings(leader, followers):
    def spacing(vehicle0, vehicle1):
        xy0 = [vehicle0.state.x, vehicle0.state.y]
        xy1 = [vehicle1.state.x, vehicle1.state.y]
        dist = math.sqrt((xy0[0] - xy1[0])**2 + (xy0[1] - xy1[1])**2)
        return dist - BACKTOWHEEL - FRONTTOWHEEL

    spacings = []
    vehicles = [leader] + followers
    for i, vehicle in enumerate(vehicles[1:]):
        spacings.append(spacing(vehicle, vehicles[i]))
    return spacings


def toggle_pause(leader_sim, follower_sims):
    leader_sim.toggle_pause_simulation()
    [follower_sim.toggle_pause_simulation() for follower_sim in follower_sims]


def wait_for_platoon_states(leader, followers):
    while (not rospy.is_shutdown() and
           (len(leader.data_handler.x) == 0
            or any([len(follower.data_handler.x) == 0 for follower in followers]))):
        pass

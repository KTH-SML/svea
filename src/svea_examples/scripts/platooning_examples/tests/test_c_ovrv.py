import pytest
import time
import rospy
import math
import numpy as np
from ..c_ovrv_utils import *
from svea.svea_managers.path_following_sveas import SVEAPlatoonMember
from svea.interfaces import LocalizationInterface
from svea.simulators.sim_SVEA import SimSVEA
from svea.models.bicycle import SimpleBicycleModel
from svea.data import RVIZPathHandler
from svea.models.cooperative import C_OVRV
from svea_msgs.msg import lli_ctrl

@pytest.fixture
def node():
    rospy.init_node("c_ovrv_test", anonymous=True)

def test_rotate2D_from_tuple():
    xy = (1.0, 0.0)
    radians = math.radians(-90)
    rot_xy = rotate2D(xy, radians)
    np.testing.assert_almost_equal(rot_xy, np.array([0.0, 1.0]))

def test_rotate2D_from_list():
    xy = [1, 0]
    radians = math.radians(-90)
    rot_xy = rotate2D(xy, radians)
    np.testing.assert_almost_equal(rot_xy, np.array([0.0, 1.0]))

def test_rotate2D_from_np():
    xy = np.array([1, 0])
    radians = math.radians(-90)
    rot_xy = rotate2D(xy, radians)
    np.testing.assert_almost_equal(rot_xy, np.array([0.0, 1.0]))

def test_compute_offset():
    spacing = 1.0 - BACKTOWHEEL - FRONTTOWHEEL
    heading = math.radians(-45)
    offset = compute_offset(spacing, heading)
    np.testing.assert_almost_equal(offset,
        np.array([math.sqrt(2.0)/2, math.sqrt(2.0)/2, 0, 0]))

def test_compute_positions():
    vehicle_pt = [0.0, 0.0, math.radians(45), 0.0]
    spacing = 1.0 - BACKTOWHEEL - FRONTTOWHEEL
    spacings = [spacing, spacing]
    leader_pt, follower_pts = compute_positions_from_spacings(vehicle_pt, spacings)
    correct_leader_pt = [math.sqrt(2.0)/2 * 2, math.sqrt(2.0)/2 * 2, math.radians(45), 0.0]
    correct_follower0_pt = [math.sqrt(2.0)/2 * 1, math.sqrt(2.0)/2 * 1, math.radians(45), 0.0]
    correct_follower1_pt = [0.0, 0.0, math.radians(45), 0.0]
    np.testing.assert_almost_equal(leader_pt, correct_leader_pt)
    np.testing.assert_almost_equal(follower_pts[0], correct_follower0_pt)
    np.testing.assert_almost_equal(follower_pts[1], correct_follower1_pt)

def test_collect_platoon_pts():
    leader_pt = [math.sqrt(2.0)/2 * 2, math.sqrt(2.0)/2 * 2, math.radians(45), 0.0]
    follower_pts = [[math.sqrt(2.0)/2 * 1, math.sqrt(2.0)/2 * 1, math.radians(45), 0.0]]
    xs, ys, yaws = collect_platoon_pts(leader_pt, follower_pts)
    assert xs == [math.sqrt(2.0)/2 * 2, math.sqrt(2.0)/2 * 1]
    assert ys == [math.sqrt(2.0)/2 * 2, math.sqrt(2.0)/2 * 1]
    assert yaws == [math.radians(45), math.radians(45)]

def test_not_reached_steady_state(node):
    steady_vel = 2.0
    leader = SVEAPlatoonMember(LocalizationInterface, [], [],
                               data_handler = RVIZPathHandler)
    follower = SVEAPlatoonMember(LocalizationInterface, [], [],
                                 data_handler = RVIZPathHandler)
    leader.state.v = 0.5
    follower.state.v = 0.5
    at_steady_state = reached_steady_state(steady_vel, leader, [follower])
    assert not at_steady_state

def test_reached_steady_state(node):
    steady_vel = 2.0
    leader = SVEAPlatoonMember(LocalizationInterface, [], [],
                               data_handler = RVIZPathHandler)
    follower = SVEAPlatoonMember(LocalizationInterface, [], [],
                                 data_handler = RVIZPathHandler)
    leader.state.v = 1.99
    follower.state.v = 1.99
    at_steady_state = reached_steady_state(steady_vel, leader, [follower])
    assert at_steady_state

def test_compute_spacings():
    leader = SVEAPlatoonMember(LocalizationInterface, [], [],
                               data_handler = RVIZPathHandler)
    follower = SVEAPlatoonMember(LocalizationInterface, [], [],
                               data_handler = RVIZPathHandler)
    leader.state.x = 1.0
    leader.state.y = 0.0
    follower.state.x = 0.5
    follower.state.y = 0.0
    spacings = compute_spacings(leader, [follower])
    assert spacings == [0.5 - BACKTOWHEEL - FRONTTOWHEEL]

def test_toggle_pause(node):
    leader_sim = SimSVEA(SimpleBicycleModel())
    follower_sim = SimSVEA(SimpleBicycleModel())
    toggle_pause(leader_sim, [follower_sim])
    assert leader_sim.is_pause
    assert follower_sim.is_pause

    toggle_pause(leader_sim, [follower_sim])
    assert not leader_sim.is_pause
    assert not follower_sim.is_pause

def test_c_ovrv_model():
    platoon_size = 1
    num_neighbors = 1 # 0 for don't use communicated info
    desired_time_headway = 0.3
    k1 = 0.5
    k2 = 0.5
    k3 = 0.8
    k4 = 0.8
    k_gains = [k1, k2, k3, k4]
    min_spacing = 0.3
    init_velocity = 1.2  # initial target velocity
    dt = 0.01

    covrv = C_OVRV(platoon_size, num_neighbors, k_gains, min_spacing,
                   desired_time_headway, init_velocity, dt=dt)
    assert np.all(covrv.A == np.array([[0.0, -1.0], [0.5, -0.65]]))
    assert np.all(covrv.b == np.array([0.0, -0.15]))
    assert np.all(covrv.equilibrium_pt == np.array([1.86, 1.2]))

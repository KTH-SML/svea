from ..c_ovrv_utils import *
from svea.svea_managers.path_following_sveas import SVEAPlatoonMember
from svea.models.cooperative import C_OVRV
import math
import numpy as np

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

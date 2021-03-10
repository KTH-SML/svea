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

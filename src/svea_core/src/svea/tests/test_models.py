#!/usr/bin/env python

"""
Test module for svea.simulators.states
"""


import unittest
import random
import sys
import math
import rospy
import std_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Twist, TwistStamped
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry
from svea_msgs.msg import lli_ctrl
from svea_msgs.msg import VehicleState as MsgVehicleState
import svea.models.states as states
from svea.models.states import VehicleState
from svea.models.states import SVEAControlValues
# from svea.models.bicycle import SimpleBicycleState


class VehicleStateTest(unittest.TestCase):

    og_kw_dict = dict(
        x=0.0,
        y=0.0,
        yaw=0.0,
        v=0.0,
        frame_id='map',
        child_frame='base_link',
        covariance=None,
        time_stamp=rospy.Time.from_sec(0)
        )

    def test_init(self):
        """Test the default initialization"""
        kw_dict = self.og_kw_dict.copy()
        state = VehicleState(**kw_dict)
        self.assertEqual(state.x, kw_dict['x'])
        self.assertEqual(state.y, kw_dict['y'])
        self.assertEqual(state.yaw, kw_dict['yaw'])
        self.assertEqual(state.v, kw_dict['v'])

    def test_init_yaw(self):
        """Test the limit of yaw"""
        kw_dict = self.og_kw_dict.copy()
        kw_dict['yaw'] = 2*math.pi
        state = VehicleState(**kw_dict)
        self.assertAlmostEqual(state.yaw, 0.0)

    def create_kw_dict(self, seed=42):
        kw_dict = self.og_kw_dict.copy()
        random.seed(seed)
        frange = lambda x: random.random()*2*x - x
        kw_dict['frame_id'] = "mock_frame"
        kw_dict['child_frame'] = "mock_base_link"
        kw_dict['time_stamp'] = frange(10)
        kw_dict['x'] = frange(10)
        kw_dict['y'] = frange(10)
        kw_dict['yaw'] = frange(math.pi)
        kw_dict['v'] = frange(10)
        return kw_dict

    def test_get_odom(self):
        """Test creating an odometry mesage from state"""
        kw_dict = self.create_kw_dict()
        state = VehicleState(**kw_dict)
        odom = state.odometry_msg

        self.assertEqual(kw_dict['frame_id'], odom.header.frame_id)
        self.assertEqual(kw_dict['child_frame'], odom.child_frame_id)
        self.assertEqual(kw_dict['time_stamp'], odom.header.stamp)
        self.assertEqual(kw_dict["x"], odom.pose.pose.position.x)
        self.assertEqual(kw_dict["y"], odom.pose.pose.position.y)
        self.assertEqual(0.0, odom.pose.pose.position.z)
        state_z = math.sin(0.5*kw_dict["yaw"])
        state_w = math.cos(0.5*kw_dict["yaw"])
        self.assertEqual(0.0, odom.pose.pose.orientation.x)
        self.assertEqual(0.0, odom.pose.pose.orientation.y)
        self.assertEqual(state_z, odom.pose.pose.orientation.z)
        self.assertEqual(state_w, odom.pose.pose.orientation.w)
        self.assertEqual(kw_dict["v"], odom.twist.twist.linear.x)

    def test_get_state(self):
        """Test creating an odometry mesage from state"""
        kw_dict = self.create_kw_dict(57)
        state = VehicleState(**kw_dict)
        state_msg = state.state_msg
        self.assertEqual(kw_dict['frame_id'], state_msg.header.frame_id)
        self.assertEqual(kw_dict['child_frame'], state_msg.child_frame_id)
        self.assertEqual(kw_dict['time_stamp'], state_msg.header.stamp)
        self.assertEqual(kw_dict["x"], state_msg.x)
        self.assertEqual(kw_dict["y"], state_msg.y)
        self.assertEqual(kw_dict["yaw"], state_msg.yaw)
        self.assertEqual(kw_dict["v"], state_msg.v)

    def test_get_pose(self):
        """Test the pose message"""
        kw_dict = self.create_kw_dict(34)
        state = VehicleState(**kw_dict)
        pose = state.pose_msg
        self.assertEqual(kw_dict['frame_id'], pose.header.frame_id)
        self.assertEqual(kw_dict['time_stamp'], pose.header.stamp)
        self.assertEqual(kw_dict["x"], pose.pose.pose.position.x)
        self.assertEqual(kw_dict["y"], pose.pose.pose.position.y)
        self.assertEqual(0.0, pose.pose.pose.position.z)
        state_z = math.sin(0.5*kw_dict["yaw"])
        state_w = math.cos(0.5*kw_dict["yaw"])
        self.assertEqual(0.0, pose.pose.pose.orientation.x)
        self.assertEqual(0.0, pose.pose.pose.orientation.y)
        self.assertEqual(state_z, pose.pose.pose.orientation.z)
        self.assertEqual(state_w, pose.pose.pose.orientation.w)

    def test_get_twist(self):
        """Test the twist message"""
        kw_dict = self.create_kw_dict(43)
        state = VehicleState(**kw_dict)
        twist = state.twist_msg
        self.assertEqual(kw_dict['child_frame'], twist.header.frame_id)
        self.assertEqual(kw_dict['time_stamp'], twist.header.stamp)
        self.assertEqual(kw_dict["v"], twist.twist.twist.linear.x)

    def populate_odom_msg(self, msg):
        """Populate a Odometry (or SVEAState) msg"""
        header = msg.header
        pose = msg.pose
        twist = msg.twist
        header.stamp = rospy.Time.from_sec(3.0)
        header.frame_id = "mock_frame"
        msg.child_frame_id = "mock_child_frame"
        pose.pose.position.x = 2.3
        pose.pose.position.y = 3.4
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 1.0
        pose.pose.orientation.w = 0.0
        twist.twist.linear.x = 2.0
        return header, pose, twist

    def test_set_odom(self):
        """Test setting from Odometry message"""
        kw_dict = self.og_kw_dict.copy()
        state = VehicleState(**kw_dict)
        odom = Odometry()
        header, pose, twist = self.populate_odom_msg(odom)
        state.odometry_msg = odom

        self.assertEqual(state.time_stamp, header.stamp)
        self.assertEqual(state.frame_id, header.frame_id)
        self.assertEqual(state.child_frame, odom.child_frame_id)
        self.assertEqual(state.x, pose.pose.position.x)
        self.assertEqual(state.y, pose.pose.position.y)
        self.assertAlmostEqual(abs(state.yaw), math.pi)
        self.assertEqual(state.v, twist.twist.linear.x)

    def test_set_state(self):
        """Test setting from twist"""
        kw_dict = self.og_kw_dict.copy()
        state = VehicleState(**kw_dict)
        state_msg = MsgVehicleState()
        state_msg.header.stamp = rospy.Time.from_sec(0.99)
        state_msg.header.frame_id = 'wupp'
        state_msg.x = 0.25
        state_msg.y = -2.3
        state_msg.yaw = 3.0
        state_msg.v = -2.2
        state.state_msg = state_msg
        self.assertEqual(state.time_stamp, state_msg.header.stamp)
        self.assertEqual(state.frame_id, state_msg.header.frame_id)
        self.assertEqual(state.child_frame, state_msg.child_frame_id)
        self.assertEqual(state.x, state_msg.x)
        self.assertEqual(state.y, state_msg.y)
        self.assertAlmostEqual(state.yaw, state_msg.yaw)
        self.assertEqual(state.v, state_msg.x)

    def test_get_array(self):
        """Test array getter"""
        kw_dict = self.og_kw_dict.copy()
        kw_dict['x'] = 30.1
        kw_dict['y'] = -23.0
        kw_dict['yaw'] = -1.5
        kw_dict['v'] = 3.0
        test_array = [kw_dict['x'], kw_dict['y'], kw_dict['yaw'], kw_dict['v']]
        state = VehicleState(**kw_dict)
        state_array = state.array
        for a, s in zip(test_array, state_array):
            self.assertEqual(a, s)

    def test_set_array(self):
        """Test setting from array"""
        kw_dict = self.og_kw_dict.copy()
        test_list = [3.01, -23.0, 0.5, 3.0]
        state = VehicleState(**kw_dict)
        for l, s in zip(test_list, state.array):
            self.assertNotEqual(l, s)
        state.array = test_list
        for l, s in zip(test_list, state.array):
            self.assertEqual(l, s)
        state.array += 0.5
        for l, s in zip(test_list, state.array):
            self.assertEqual(l+0.5, s)
        state.array += 3
        self.assertNotEqual(test_list[2]+3.5, state.yaw)

    def test_iterate(self):
        """Test itterating through a state"""
        kw_dict = self.og_kw_dict.copy()
        kw_dict['x'] = 30.1
        kw_dict['y'] = -23.0
        kw_dict['yaw'] = -1.5
        kw_dict['v'] = 3.0
        test_array = [kw_dict['x'], kw_dict['y'], kw_dict['yaw'], kw_dict['v']]
        state = VehicleState(**kw_dict)
        for a, s in zip(test_array, state):
            self.assertEqual(a, s)

    def test_set_state_msg(self):
        """Test itterating through a state"""
        kw_dict = self.og_kw_dict.copy()
        kw_dict['x'] = 30.1
        kw_dict['y'] = -23.0
        kw_dict['yaw'] = -1.5
        kw_dict['v'] = 3.0
        test_array = [kw_dict['x'], kw_dict['y'], kw_dict['yaw'], kw_dict['v']]
        state = VehicleState(**kw_dict)
        for a, s in zip(test_array, state):
            self.assertEqual(a, s)

    def test_get_dict(self):
        """Test getting the state as a dict"""
        kw_dict = self.create_kw_dict(77)
        state = VehicleState(**kw_dict)
        state_dict = state.dict
        self.assertEqual(kw_dict['x'], state_dict['x'])
        self.assertEqual(kw_dict['y'], state_dict['y'])
        self.assertEqual(kw_dict['yaw'], state_dict['yaw'])
        self.assertEqual(kw_dict['v'], state_dict['v'])

    def test_state_len(self):
        """Test getting number of contionus states"""
        kw_dict = self.create_kw_dict(82)
        state = VehicleState(**kw_dict)
        self.assertEqual(4, len(state))


class HelperFunctionTest(unittest.TestCase):
    def test_quaternion_to_yaw(self):
        quat = [0.0, 0.0, 0.0, 1.0]
        yaw = states.xy_yaw_from_quaternion(quat)
        self.assertAlmostEqual(yaw, 0.0)

        wanted_yaw = math.pi/4.5
        z = math.sin(0.5*wanted_yaw)
        w = math.cos(0.5*wanted_yaw)
        quat = [0.0, 0.0, z, w]
        yaw = states.xy_yaw_from_quaternion(quat)
        self.assertAlmostEqual(yaw, wanted_yaw)

        quat = [0.0, 0.0, 1.0, 0.0]
        yaw = states.xy_yaw_from_quaternion(quat)
        self.assertAlmostEqual(yaw, math.pi)

        quat = [0.0, 0.0, -1.0, 0.0]
        yaw = states.xy_yaw_from_quaternion(quat)
        self.assertAlmostEqual(yaw, math.pi)


class ControlValuesTest(unittest.TestCase):

    og_kw_dict = dict(
        steering=0,
        velocity=0,
        gear=0,
        front_diff_locked=False,
        rear_diff_locked=False,
        control_flags=0,
    )

    def test_create_default(self):
        values = SVEAControlValues()
        self.assertEqual(values.steering, -SVEAControlValues.valid_range)
        self.assertEqual(values.velocity, -SVEAControlValues.valid_range)
        self.assertTrue(values.gear is None)
        self.assertTrue(values.front_diff_locked is None)
        self.assertTrue(values.rear_diff_locked is None)
        self.assertEqual(values.control_flags, 0)

    def _get_kw_dict(self, seed=42):
        kw_dict = self.og_kw_dict.copy()
        random.seed(seed)

        def frange(x):
            return random.random()*2*x - x
        kw_dict['steering'] = frange(127)
        kw_dict['velocity'] = frange(127)
        kw_dict['gear'] = round(abs(frange(1)))
        kw_dict['front_diff_locked'] = random.random() > 0.5
        kw_dict['rear_diff_locked'] = random.random() > 0.5
        kw_dict['control_flags'] = frange(127)
        return kw_dict

    def test_create_with_values(self):
        kw_dict = self._get_kw_dict()
        values = SVEAControlValues(**kw_dict)
        self.assertEqual(values.steering, int(kw_dict['steering']))
        self.assertEqual(values.velocity, int(kw_dict['velocity']))
        self.assertEqual(values.gear, kw_dict['gear'])
        self.assertEqual(values.front_diff_locked, kw_dict['front_diff_locked'])
        self.assertEqual(values.rear_diff_locked, kw_dict['rear_diff_locked'])
        self.assertEqual(values.control_flags, kw_dict['control_flags'])

    def test_create_msg_from_empty(self):
        values = SVEAControlValues()
        msg = values.control_msg
        self.assertEqual(msg.steering, -128)
        self.assertEqual(msg.velocity, -128)
        self.assertEqual(msg.trans_diff, 0)
        self.assertEqual(msg.ctrl, 0)

    def test_update_from_msg(self):
        kw_dict = self._get_kw_dict()
        values = SVEAControlValues(**kw_dict)
        msg = values.control_msg
        changed = values.update_from_msg(msg)
        self.assertFalse(changed)
        msg.steering = 22
        msg.velocity = -22
        changed = values.update_from_msg(msg)
        self.assertTrue(changed)
        self.assertEqual(values.steering, 22)
        self.assertEqual(values.velocity, -22)
        self.assertEqual(values.gear, 0)
        self.assertEqual(values.front_diff_locked, False)
        self.assertEqual(values.rear_diff_locked, False)

        msg.trans_diff = 0b00000111
        changed = values.update_from_msg(msg)
        self.assertFalse(changed)
        self.assertEqual(values.gear, 0)
        self.assertEqual(values.front_diff_locked, False)
        self.assertEqual(values.rear_diff_locked, False)

        msg.trans_diff = 0b00111111
        changed = values.update_from_msg(msg)
        self.assertTrue(changed)
        self.assertEqual(values.gear, 1)
        self.assertEqual(values.front_diff_locked, True)
        self.assertEqual(values.rear_diff_locked, True)


def suite():
    """Agregate all test cases"""
    suite = unittest.TestSuite()
    suite.addTest(VehicleStateTest)
    # suite.addTest(BicycleModelTest)
    suite.addTest(HelperFunctionTest)
    suite.addTest(ControlValuesTest)
    return suite


PKG = 'test_models'


if __name__ == '__main__':
    #import rosunit
    #rosunit.unitrun(PKG, 'test_states', TestStates)
    import rostest
    test_suite = suite()
    rostest.rosrun(PKG, 'test_states', test_suite)

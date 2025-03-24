
"""Classes for representing the state of a ground vehicle.

TODO:
    * Implement handling of covaraiances
    * Handling of state messages
"""

import math
import rclpy
from rclpy.node import Node
import numpy as np
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance, PoseStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import svea_msgs.msg
from svea_msgs.msg import LLIControl as lli_ctrl

__license__ = "MIT"
__maintainer__ = "Tobias Bolin"
__email__ = "tbolin@kth.se"
__status__ = "Development"


class SVEAControlValues(object):
    """
    Convenience class for representing SVEA control values in a human readable format.

    The default values corresponds to the do not care values for the LLI.
    That is: if a ctrl_lli message is generated from a `SVEAControlValues` instance
    that was initiated without arguments nothing will change if it is sent to the LLI.

    :param steering: Steering, defaults to -128
    :type steering: int, optional
    :param velocity: Velocity, defaults to -128
    :type velocity: int, optional
    :param gear: Gear, range 0 or 1, defaults to None
    :type steering: int or None, optional
    :param front_diff_locked: Gear, defaults to None
    :type steering: bool or None, optional
    :param rear_diff_locked: Gear, defaults to None
    :type steering: bool or None, optional
    """

    valid_range = 128
    gear_mask = 0b00000001
    gear_change_mask = 0b00001000
    fdiff_mask = 0b00000010
    fdiff_change_mask = 0b00010000
    rdiff_mask = 0b00000100
    rdiff_change_mask = 0b00100000

    def __init__(self,
                 steering=-128, velocity=-128,
                 gear=None,
                 front_diff_locked=None, rear_diff_locked=None,
                 control_flags=0b00000000):
        self._steering = 0
        self.steering = steering
        self._velocity = 0
        self.velocity = velocity
        self._gear = 0
        self.gear = gear # Set through property
        self.front_diff_locked = front_diff_locked
        self.rear_diff_locked = rear_diff_locked
        self.control_flags = control_flags

    def _diff_status_as_text(self, diff_status):
        if diff_status is None:
            return 'DNC'
        if diff_status:
            return 'Locked'
        else:
            return 'Unlocked'

    def _build_param_printout(self):
        """Build a string representation of the state suitable for printing"""
        status = dict(
            steering=self.steering if abs(self.steering) < self.valid_range else 'DNC',
            velocity=self.velocity if abs(self.velocity) < self.valid_range else 'DNC',
            gear_status="Low" if self.gear == 0 else "High" if self.gear == 1 else 'DNC',
            front_diff_status=self._diff_status_as_text(self.front_diff_locked),
            rear_diff_status=self._diff_status_as_text(self.front_diff_locked),
            flags='0b' + '0'*(10-len(bin(self.control_flags))) + bin(self.control_flags)[2:],
        )   # The flags hack just pads the flags string to 8 bits
        param_str = ("  -  steering: {steering}\n"
                     "  -  velocity: {velocity}\n"
                     "  -  gear       - {gear_status}\n"
                     "  -  diff_front - {front_diff_status}\n"
                     "  -  diff_rear  - {rear_diff_status}\n"
                     "  -  flags      - {flags}"
                     ).format(**status)
        return param_str

    def __repr__(self):
        return "Control Values:\n" + self._build_param_printout()

    def __str__(self):
        return self._build_param_printout()

    def _interpret_value(self, value):
        if abs(value) < self.valid_range:
            return int(value)
        else:
            return -self.valid_range

    @property
    def steering(self):
        """ Steering actuation value.

        The valid range is [-127, 127].
        Anything else will be interpreted as a Do Not Care

        :getter: Return the steering value.
        :setter: Set the current steering value.
        :type: int
        """
        return self._steering

    @steering.setter
    def steering(self, new_steering):
        self._steering = int(self._interpret_value(new_steering))

    @property
    def velocity(self):
        """ Velocity actuation value.

        The valid range is [-127, 127].
        Anything else will be interpreted as a Do Not Care

        :getter: Return the velocity value.
        :setter: Set the current velocity value.
        :type: int
        """
        return self._velocity

    @velocity.setter
    def velocity(self, new_velocity):
        self._velocity = self._interpret_value(new_velocity)

    @property
    def gear(self):
        """ Gear of the vehicle. Can be either 0 (low) or 1 (high)

        :getter: Return the current gear
        :setter: Set the current gear (0 or 1)
        :type: int or None
        """
        return self._gear

    @gear.setter
    def gear(self, gear_number):
        if gear_number in (0, 1) or gear_number is None:
            self._gear = gear_number
        else:
            raise ValueError('Gear should only be 0, 1 or None, not {}'.format(gear_number))

    @property
    def control_msg(self):
        """ The control value interpreted as a lli_ctrl message.

        :getter: A lli_ctrl message representing the current values
        :setter: Set the current values from a lli_ctr message
        :type: class: `svea_msg.msgs.lli_ctrl`
        """

        ctrl_msg = lli_ctrl()
        ctrl_msg.steering = self._interpret_value(self.steering)
        ctrl_msg.velocity = self._interpret_value(self.velocity)
        trans_diff = 0
        statuses = (self.gear, self.front_diff_locked, self.rear_diff_locked)
        for i, status in enumerate(statuses):
            if status is None:
                continue
            trans_diff += 1 << (3+i)
            trans_diff += 1 << (i) if status else 0
        ctrl_msg.trans_diff = trans_diff
        ctrl_msg.ctrl = self.control_flags
        return ctrl_msg

    @control_msg.setter
    def ctrl_msg(self, ctrl_msg):
        self.steering = ctrl_msg.steering
        self.velocity = ctrl_msg.velocity
        status = ctrl_msg.trans_diff
        if status & self.gear_change_mask:
            gear_is_high = status & self.gear_mask
            self._gear = 1 if gear_is_high else 0
        else:
            self._gear = None
        if status & self.fdiff_change_mask:
            self.front_diff_locked = status & self.fdiff_mask
        else:
            self.front_diff_locked = None
        if status & self.rdiff_change_mask:
            self.rear_diff_locked = status & self.rdiff_mask
        else:
            self.rear_diff_locked = None
        self.control_flags = ctrl_msg.ctrl

    def update_from_msg(self, ctrl_msg):
        """"Update the class from a control message.

        Unlike setting the control message directly this method
        respects the don not care rules according to the same logic as the LLI.
        Usefull for interpretting incoming control messages

        :param ctrl_msg: Yaw of the vehicle in radians
        :type ctrl_msg: class: `svea_msg.msgs.lli_ctrl`
        :return: True if the control message changed anything, False otherwise
        :rtype: bool
        """

        prev_steer = self._steering
        prev_vel = self._velocity
        prev_gear = self._gear
        prev_fdiff = self.front_diff_locked
        prev_rdiff = self.rear_diff_locked
        prev_flags = self.control_flags

        if abs(ctrl_msg.steering) < self.valid_range:
            self.steering = ctrl_msg.steering
        if abs(ctrl_msg.velocity) < self.valid_range:
            self.velocity = ctrl_msg.velocity
        status = ctrl_msg.trans_diff
        if status & self.gear_change_mask:
            gear_is_high = status & self.gear_mask
            self._gear = 1 if gear_is_high else 0
        if status & self.fdiff_change_mask:
            self.front_diff_locked = status & self.fdiff_mask
        if status & self.rdiff_change_mask:
            self.rear_diff_locked = status & self.rdiff_mask
        if ctrl_msg.ctrl != self.control_flags:
            self.control_flags = ctrl_msg.ctrl

        changed = (prev_steer != self.steering
                   or prev_vel != self.velocity
                   or prev_gear != self.gear
                   or prev_fdiff != self.front_diff_locked
                   or prev_rdiff != self.rear_diff_locked
                   or prev_flags != self.control_flags)
        return changed


## Helper functions ##
def _set_placehoder_covariance(covariance):
    """Fill the list covariance so it is similar to a identity matrix"""
    for i in range(0, len(covariance), int(len(covariance)**0.5)+1):
        covariance[i] = 1.0


def _normalize_angle(angle):
    """Normalize an angle to the interval (-pi, pi]"""
    while angle > np.pi:
        angle -= 2*np.pi
    while angle <= -np.pi:
        angle += 2*np.pi
    return angle


def rotate_xy(xy_pt, rot_angle):
    """Rotate numpy array xy_pt by rot_angle"""
    c = math.cos(rot_angle)
    s = math.sin(rot_angle)
    rot_mat = np.array([[c, -s], [s, c]])
    pt = np.reshape(xy_pt, (2, 1))
    rot_pt = rot_mat @ pt
    return np.reshape(rot_pt, xy_pt.shape)


def yaw_cov_to_quaternion_cov(yaw, yaw_covariance):
    """Calculate the quaternion covariance based on the yaw and yaw covariance.

    Perform the operation :math:`C_{\\theta} = R C_q R^T`
    where :math:`C_{\\theta}` is the yaw covariance,
    :math:`C_q` is the quaternion covariance and :math:`R` is
    the Jacobian of the transform from yaw to a quaternion.
    :math:`R` will be a collumn vector defined by:

        .. math::
            R = \\\\
            \\frac{dx}{d\\theta}     &= 0, \\\\
            \\frac{dy}{d\\theta}     &= 0, \\\\
            \\frac{dz}{d\\theta}     &= \\frac{1}{2} \\cos \\frac{1}{2} \\theta, \\\\
            \\frac{dw}{d\\theta}     &= -\\frac{1}{2} \\sin \\frac{1}{2} \\theta, \\\\

    :param yaw: Yaw of the vehicle in radians
    :type quat: float
    :return: The yaw covariance transformed to quaternion coordinates.
    :rtype: 4x4 numpy array
    """
    R = np.c_[0,
              0,
              0.5*math.cos(yaw*0.5),
              -0.5*math.sin(yaw*0.5)].T
    quat_covariance = R.dot(yaw_covariance).dot(R.T)
    return quat_covariance


def quaternion_cov_to_yaw_cov(quat, quat_covariance):
    """Calculate the quaternion covariance based on the yaw and yaw covariance.

    Perform the operation :math:`C_q = R C_{\\theta} R^T`
    where :math:`C_q` is the quaternion covariance,
    :math:`C_{\\theta}` is the yaw covariance and :math:`R` is
    the Jacobian of the transform from a quaternion into yaw.
    :math:`R` will be a row vector defined by:

        .. math::
            R = \\\\
            \\frac{\\partial \\theta}{\\partial x}     &= ay + bx, \\\\
            \\frac{\\partial \\theta}{\\partial y}     &= ax - by, \\\\
            \\frac{\\partial \\theta}{\\partial z}     &= aw - bz, \\\\
            \\frac{\\partial \\theta}{\\partial w}     &= az + bw, \\\\

    The transform from yaw to quaternion is :math:`\\theta = arctan2(g, h)`,
    with :math:`g = 2zw + 2xy` and :math:`h = x^2 + y^2 + z^2 + w^2`.
    The partial derivatives of  :math:`\\arctan2(g, h)` in terms of
    :math:`g` and `h` are

        .. math::
            \\frac{\\partial}{\\partial g}    &= \\frac{g}{g^2+h^2}, \\\\
            \\frac{\\partial}{\\partial h}    &= \\frac{h}{g^2+h^2}, \\\\

    With :math:`a = 2frac{\\partial}{\\partial g}` and `b = 2frac{\\partial}{\\partial h}`,
    the chain rule yields the expression for :math:`R` above.

    :param quat: Heading of the vehicle represented by a quaternion.
    :type quat: Any container that has numerical values in index 0, 1, 2 and 3
    :param quat_covariance: Covariance matrix for the quaternion
    :type quat_covariance: 4x4 numpy array
    :return: The quaternion covariance transformed to yaw coordinates.
    :rtype: float
    """
    x, y, z, w = quat[:4]
    g = 2*z*w + 2*x*y
    h = x**2 + y**2 + z**2 + w**2
    gh2 = g**2 + h**2
    a = 2*g/gh2
    b = 2*h/gh2

    R = np.c_[a*y + b*x, a*x - b*y, a*w - b*z, a*z + b*w]
    yaw_covariance = R.dot(quat_covariance).dot(R.T)
    return yaw_covariance


def xy_yaw_from_quaternion(quat):
    """Calculate the yaw angle in the xy plane from a rotation quaternion.
    :param quat: A unit quaternion representing the
    :type quat: Any container that has numerical values in index 0, 1, 2 and 3
    :return: The yaw angle in radians projected on the xy plane
    :rtype: float
    """
    x, y, z, w = quat[:4]
    return math.atan2(2*z*w + 2*x*y, w**2 + x**2 - y**2 - z**2)


def extract_pose(msg):
    """ Extract the pose from a message
    :param msg: An Odometry, Subject, Transform or any variation of Pose message
    :return: The Pose contained in msg
    :rtype: class:`geometry_msgs.msg.Pose`
    """
    if isinstance(msg, (Odometry, PoseWithCovarianceStamped)):
        return msg.pose.pose
    elif isinstance(msg, (PoseStamped, PoseWithCovariance)):
        return msg.pose
    elif isinstance(msg, Pose):
        return msg
    else:
        raise TypeError("Could not extract pose from message of type {}".format(type(msg)))


def extract_pose_covariance(msg):
    """ Extract the covariance from a message
    :param msg: An Odometry, Subject, Transform or any variation of Pose message
    :return: The Covariance contained in msg
    :rtype: class:`numpy.array`
    """
    if isinstance(msg, (Odometry, PoseWithCovarianceStamped)):
        covaraiance_list = msg.pose.covariance
    elif isinstance(msg, PoseWithCovariance):
        covaraiance_list = msg.covariance
    else:
        raise TypeError("Could not extract pose covariance from message of type {}".format(type(msg)))
    covariance_array = np.array(covaraiance_list).reshape(6, 6)
    return covariance_array


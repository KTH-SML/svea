
"""Classes for representing the state of a ground vehicle.

TODO:
    * Implement handling of covaraiances
    * Handling of state messages
"""

import math
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance, PoseStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import svea_msgs.msg
from svea_msgs.msg import lli_ctrl

__license__ = "MIT"
__maintainer__ = "Tobias Bolin"
__email__ = "tbolin@kth.se"
__status__ = "Development"


class VehicleState(object):
    """
    A 2D state for vehicles
    Units are [m, rad, s, m/s]

    :param x: Initial x position, defaults to 0.0
    :type x: float, optional
    :param y: Initial y position, defaults to 0.0
    :type y: float, optional
    :param yaw: Initial yaw, defaults to 0.0
    :type yaw: float, optional
    :param v: Initial velocity, defaults to 0.0
    :type v: float, optional
    :param gear: Initial gear, can be 0 (low) or 1 (high), defaults to 0
    :type gear: int, optional
    :param front_diff_lock: Initial front differential lock status, defaults to `False`
    :type front_diff_lock: bool, optional
    :param rear_diff_lock: Initial rear differential lock status, defaults to `False`
    :type rear_diff_lock: bool, optional
    :param frame_id: Reference frame for the state, defaults to `map`
    :type frame_id: string, optional
    :param child_frame: Child frame for the state, defaults to `base_link`
    :type child_frame: string, optional
    :param covariance: Covariance matrix 4x4, defaults to a unity matrix
    :type covariance: numpy.array, optional
    :param time_stamp: Time stamp for the state, defaults to `rospy.Time.now()`
    :type time_stamp: rospy.Time, optional
    """

    _NUMBER_OF_STATES = 4
    X_IX = 0
    Y_IX = 1
    YAW_IX = 2
    V_IX = 3

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0,
                 frame_id='map',
                 child_frame='base_link',
                 covariance=None,
                 time_stamp=rospy.Time.from_sec(0)):
        super(VehicleState, self).__init__()
        self._x = 0.0
        self.x = x
        self._y = 0.0
        self.y = y
        self._yaw = 0.0
        self.yaw = yaw
        self._v = 0.0
        self.v = v
        self.frame_id = frame_id
        self.child_frame = child_frame
        self._covariance = np.eye(self._NUMBER_OF_STATES)
        self.covariance = covariance # Set through property
        self.time_stamp = time_stamp
        # Lazy message initialization
        self._msgs_are_initialized = False
        self._msgs_are_updated = False
        self._pose_is_updated = False
        self._covariance_is_updated = False
        self._state_msg = None
        self._pose_msg = None
        self._twist_msg = None
        self._odometry_msg = None

    def _printable_covariance(self):
        """Create a print friendly string representation of the covariance"""
        with np.printoptions(precision=4,
                             suppress=True,
                             formatter={'float': '{:0.4f}'.format},
                             linewidth=80):
            return self.covariance.__repr__()

    def _build_param_printout(self):
        """Build a string representation of the state suitable for printing"""
        status = dict(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            v=self.v,
            printable_covariance=self._printable_covariance()
        )
        status.update(self.__dict__)
        param_str = ("  -   x: {x}\n"
                     "  -   y: {y}\n"
                     "  - yaw: {yaw}\n"
                     "  -   v: {v}\n"
                     "  -  frame_id   - {frame_id}\n"
                     "  -  child_frame- {child_frame}\n"
                     "  -  covariance - {printable_covariance}\n"
                     "  -  time stamp: {time_stamp}").format(**status)
        return param_str

    def __repr__(self):
        return "## Vehicle State:\n" + self._build_param_printout()

    def __str__(self):
        return self._build_param_printout()

    def __iter__(self):
        return iter((self.x, self.y, self.yaw, self.v))

    @property
    def x(self):
        """
        X position of the vehicle in [m] with respect to `frame_id`

        :getter: Return the current x position in [m]
        :setter: Set the current x position in [m]
        :type: float
        """
        return self._x

    @x.setter
    def x(self, new_x):
        self._x = new_x
        self._pose_is_updated = False

    @property
    def y(self):
        """
        Y position of the vehicle in [m] with respect to `frame_id`

        :getter: Return the current y position in [m]
        :setter: Set the current y position in [m]
        :type: float
        """
        return self._y

    @y.setter
    def y(self, new_y):
        self._y = new_y
        self._pose_is_updated = False

    @property
    def yaw(self):
        """
        Yaw of the vehicle in [rad] with respect to `frame_id`

        :getter: Return the current yaw in [rad]
        :setter: Set the current yaw in [rad]
        :type: float
        """
        return self._yaw

    @yaw.setter
    def yaw(self, new_yaw):
        self._yaw = _normalize_angle(new_yaw)
        self._pose_is_updated = False

    @property
    def v(self):
        """
        Velocity of the vehicle in [m/s] with respect to `frame_id`

        :getter: Return the current velocity in [m/s]
        :setter: Set the current velocity in [m/s]
        :type: float
        """
        return self._v

    @v.setter
    def v(self, new_v):
        self._v = new_v
        self._twist_is_updated = False

    @property
    def covariance(self):
        """Covariance of the state, should be a 4x4 matrix"""
        return self._covariance

    @covariance.setter
    def covariance(self, new_covariance):
        shape = (self._NUMBER_OF_STATES, self._NUMBER_OF_STATES)
        if new_covariance is None:
            new_covariance = np.eye(self._NUMBER_OF_STATES)
        else:
            new_covariance = np.array(new_covariance).reshape(*shape)
        if new_covariance.shape != shape:
            raise AttributeError('Covaraince have to be a {0}x{0} matrix, is {1}'
                                 .format(self._NUMBER_OF_STATES, new_covariance.shape))
        else:
            self._covariance = new_covariance
        self._covariance_is_updated = False

    def _initialize_msgs(self):
        """Initialize messages when they are needed"""
        self._state_msg = svea_msgs.msg.VehicleState()
        self._odometry_msg = Odometry()
        self._pose_msg = PoseWithCovarianceStamped()
        self._pose_msg.pose = self._odometry_msg.pose
        self._pose_msg.header = self._state_msg.header
        self._state_msg.child_frame_id = self.child_frame
        self._odometry_msg.child_frame_id = self.child_frame
        self._odometry_msg.header = self._state_msg.header
        self._twist_msg = TwistWithCovarianceStamped()
        self._twist_msg.twist = self._odometry_msg.twist
        _set_placehoder_covariance(self._twist_msg.twist.covariance)
        self._msgs_are_initialized = True
        # self._msgs_are_updated = False
        self._pose_is_updated = False
        self._twist_is_updated = False
        self._covariance_is_updated = False

    def _build_msg_header(self, msg):
        """Update a message header"""
        msg.header.stamp = self.time_stamp
        msg.header.frame_id = self.frame_id

    def _build_state_msg(self):
        """Update the odometry, pose and twist messages"""
        if not self._msgs_are_initialized:
            self._initialize_msgs()
        self._build_msg_header(self._state_msg)
        self._state_msg.x = self.x
        self._state_msg.y = self.y
        self._state_msg.yaw = self.yaw
        self._state_msg.v = self.v
        self._state_msg.covariance = list(self.covariance.flatten())
        return self._state_msg

    def _build_covariances(self):
        position_cov = self.covariance[self.X_IX:self.Y_IX, self.X_IX:self.Y_IX]
        pose_cov = np.eye(6) # 6 states in a ROS pose covariance
        pose_cov[self.X_IX:self.Y_IX, self.X_IX:self.Y_IX] = position_cov
        yaw_cov = self.covariance[self.YAW_IX, self.YAW_IX]
        pose_cov[5, 5] = yaw_cov
        self._pose_msg.pose.covariance = list(pose_cov.flatten())
        self._twist_msg.twist.covariance[0] = self.covariance[self.V_IX, self.V_IX]
        self._covariance_is_updated = True

    def _build_pose_msg(self):
        """Update the pose message"""
        if not self._msgs_are_initialized:
            self._initialize_msgs()
        if not self._pose_is_updated:
            self._build_msg_header(self._pose_msg)
            pose = self._pose_msg.pose
            pose.pose.position.x = self.x
            pose.pose.position.y = self.y
            q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, self.yaw)
            pose.pose.orientation.x = q_x
            pose.pose.orientation.y = q_y
            pose.pose.orientation.z = q_z
            pose.pose.orientation.w = q_w
            self._pose_is_updated = True
        if not self._covariance_is_updated:
            self._build_covariances()
        return self._pose_msg

    def _build_twist_msg(self):
        """Update the twist message"""
        if not self._msgs_are_initialized:
            self._initialize_msgs()
        if not self._twist_is_updated:
            self._twist_msg.header.stamp = self.time_stamp
            self._twist_msg.header.frame_id = self.child_frame
            twist = self._twist_msg.twist
            twist.twist.linear.x = self.v
            self._twist_is_updated = True
        if not self._covariance_is_updated:
            self._build_covariances()
        return self._twist_msg

    def _build_odometry_msg(self):
        """Update the odometry, pose and twist messages"""
        if not self._msgs_are_initialized:
            self._initialize_msgs()
        self._build_msg_header(self._odometry_msg)
        self._odometry_msg.child_frame_id = self.child_frame
        self._build_pose_msg()
        self._build_twist_msg()
        return self._odometry_msg

    @property
    def pose_msg(self):
        """Pose msg representing the state"""
        self._build_pose_msg()
        return self._pose_msg

    @property
    def twist_msg(self):
        """Twist msg representing the state"""
        self._build_twist_msg()
        return self._twist_msg

    @property
    def odometry_msg(self):
        """Odometry msg representing the state"""
        self._build_odometry_msg()
        return self._odometry_msg

    @odometry_msg.setter
    def odometry_msg(self, new_odom):
        if not self._msgs_are_initialized:
            self._initialize_msgs()
        self.time_stamp = new_odom.header.stamp
        self.frame_id = new_odom.header.frame_id
        self.child_frame = new_odom.child_frame_id
        self._update_from_pose_msg(new_odom.pose)
        self._update_from_twist_msg(new_odom.twist)

    def _update_from_header(self, new_header):
        self.frame_id = new_header.frame_id
        self.time_stamp = new_header.stamp

    @property
    def state_msg(self):
        """SVEAState msg representing the state of the vehicle"""
        self._build_state_msg()
        return self._state_msg

    @state_msg.setter
    def state_msg(self, new_state):
        self._update_from_header(new_state.header)
        self.x = new_state.x
        self.y = new_state.y
        self.yaw = new_state.yaw
        self.v = new_state.v

    def _update_from_pose_msg(self, pose_msg):
        pose = extract_pose(pose_msg)
        self.x = pose.position.x
        self.y = pose.position.y
        quat = [pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w]
        self.yaw = xy_yaw_from_quaternion(quat)
        try:
            pose_cov = extract_pose_covariance(pose_msg)
            x_ix = self.X_IX
            y_ix = self.Y_IX
            self.covariance[x_ix:y_ix, x_ix:y_ix] = pose_cov[x_ix:y_ix, x_ix:y_ix]
            self.covariance[self.YAW_IX, self.YAW_IX] = pose_cov[5, 5]
            self._covariance_is_updated = False
        except TypeError:
            pass

    def _update_from_twist_msg(self, twist_msg):
        twist = twist_msg
        while True:
            try:
                twist = twist.twist
            except AttributeError:
                break
        # project velocity vector onto heading of vehicle
        rot_angle = -self.yaw # rotate to heading of body frame
        vel_vec = np.array([twist.linear.x, twist.linear.y])
        rot_vel_vec = rotate_xy(vel_vec, rot_angle)
        # take x-component of velocity vector in body frame
        self.v = rot_vel_vec[0]
        try:
            self.covariance[self.V_IX, self.V_IX] = twist_msg.covariance[0]
        except AttributeError:
            self.covariance[self.V_IX, self.V_IX] = twist_msg.twist.covariance[0]

    @property
    def array(self):
        """ An array representing the continous state [x, y, yaw, v] of the vehicle.

        :getter: A numpy array with the state of the vehicle [x, y, yaw, v].
        :setter: Set the current state [x, y, yaw, v] from an array.
        :type: numpy.array
        """
        return np.r_[self.x, self.y, self.yaw, self.v]

    @array.setter
    def array(self, array):
        try:
            self.x, self.y, self.yaw, self.v = array
        except ValueError: # For nested arrays i.e. matrices
            self.x, self.y, self.yaw, self.v = array.squeeze()

    @property
    def dict(self):
        """The state represented by a dictionary"""
        state_dict = dict(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            v=self.v,
        )
        return state_dict

    def __len__(self):
        return self._NUMBER_OF_STATES


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

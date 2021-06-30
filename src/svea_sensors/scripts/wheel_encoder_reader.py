#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from svea.sensors import WheelEncoder


""" Node for reading a wheel encoder
"""


__author__ = "Tobias Bolin"
__copyright__ = "Copyright 2020, Tobias Bolin"
__credits__ = ["Tobias Bolin"]
__license__ = "MIT"
__version__ = "0.0.1"
__maintainer__ = "Tobia Bolin"
__email__ = "tbolin@kth.se"
__status__ = "Development"


# Axle track in mm
DEFAULT_AXLE_TRACK = 199.0
# Wheel radius in mm
DEFAULT_WHEEL_RADIUS = 60.0
TICKS_PER_REVOLUTION = 60

DEFAULT_LINEAR_COVARIANCE = 0.2
DEFAULT_ANGULAR_COVARIANCE = 0.4


def set_covariance(cov_matrix, linear_covariance, angular_covariance):
    """Setup the covariance matrix of the twist message"""
    cov_step = 7 # Index step in covariance matrix
    lin_cov_end = cov_step + 1
    for i in range(0, lin_cov_end, cov_step):
        cov_matrix[i] = linear_covariance
    ang_cov_start = lin_cov_end + cov_step - 1
    ang_cov_end = lin_cov_end + 3*cov_step
    for i in range(ang_cov_start, ang_cov_end, cov_step):
        cov_matrix[i] = angular_covariance


def main():
    rospy.init_node('wheel_encoder_reader', anonymous=False)
    arg_dict = dict(
        encoder_frame=rospy.get_param('~encoder_frame', 'base_link'),
        encoder_topic=rospy.get_param('~encoder_topic', 'lli/encoder'),
        direction_topic=rospy.get_param('~direction_topic', 'actuation_twist'),
        axle_track=rospy.get_param('axle_track', DEFAULT_AXLE_TRACK),
        wheel_radius=rospy.get_param('wheel_radius', DEFAULT_WHEEL_RADIUS),
        linear_covariance=rospy.get_param('linear_covariance', DEFAULT_LINEAR_COVARIANCE),
        angular_covariance=rospy.get_param('angular_covariance', DEFAULT_ANGULAR_COVARIANCE),
    )
    encoder_interface = WheelEncoder(**arg_dict)

    twist_publisher = rospy.Publisher(
        'wheel_encoder_twist',
        TwistWithCovarianceStamped,
        queue_size=1,
        tcp_nodelay=True)

    twist_msg = TwistWithCovarianceStamped()
    set_covariance(
        twist_msg.twist.covariance,
        linear_covariance=encoder_interface.linear_covariance,
        angular_covariance=encoder_interface.angular_covariance,
    )
    twist_msg.header.frame_id = encoder_interface.frame_id

    def publish_twist(wheel_encoder):
        twist_msg.header.stamp = rospy.Time.now()
        twist = twist_msg.twist.twist
        twist.linear.x = wheel_encoder.linear_velocity
        twist.angular.z = wheel_encoder.angular_velocity
        twist_publisher.publish(twist_msg)

    encoder_interface.add_callback(publish_twist)
    encoder_interface.start()
    rospy.spin()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
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


def set_covariance(linear_covariance, angular_covariance):
    """Setup the covariance matrix of the twist message"""
    # sigma_yy = 0 given svea's kinematics
    cov_matrix = [  linear_covariance, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, linear_covariance, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, angular_covariance, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, angular_covariance, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, angular_covariance]
    return cov_matrix

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
    twist_msg.twist.covariance = set_covariance(
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
#!/usr/bin/env python
import math
import rospy
from svea_sensors import wheel_encoder_interface

from svea_msgs.msg import lli_encoder
from geometry_msgs.msg import TwistWithCovarianceStamped

def main():
    encoder_interface = wheel_encoder_interface.WheelEncoderInterface()
    rospy.init_node(encoder_interface.node_name)
    encoder_interface.start(block=True)

if __name__ == '__main__': main()

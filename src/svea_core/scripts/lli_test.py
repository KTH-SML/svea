#!/usr/bin/env python3

"""
Node to test the lli control interface by giving different directional commands to the car.
"""

import rospy
import math
from svea.interfaces import ActuationInterface

def main():
    """
    Starts instance of ControlInterface and waits for initialization.
    Sends move forward, move backward, turn left and turn right commands to the car for 3 seconds each.
    """
    rospy.init_node('lli_testing')
    ctrl_interface = ActuationInterface().start(wait = True)
    rate = 10
    r = rospy.Rate(rate)
    run_time = 3
    test_speed = 0.3 # [m/s]
    test_angle = math.pi/8 # [rad] approx 22.5 degrees

    rospy.loginfo("Going forward for 3 seconds")
    i = 0
    while not rospy.is_shutdown() and i < rate*run_time:
        ctrl_interface.send_control(
                steering=0,
                velocity=test_speed,
                transmission=0)
        rospy.loginfo_throttle(run_time, ctrl_interface)
        r.sleep()
        i += 1

    rospy.loginfo("Going backwards for 3 seconds")
    i = 0
    while not rospy.is_shutdown() and i < rate*run_time:
        ctrl_interface.send_control(0, -test_speed)
        rospy.loginfo_throttle(run_time, ctrl_interface)
        r.sleep()
        i += 1

    i = 0
    rospy.loginfo("Braking for 3 seconds")
    while not rospy.is_shutdown() and i < rate*run_time:
        ctrl_interface.send_control(brake_force=20)
        rospy.loginfo_throttle(run_time, ctrl_interface)
        r.sleep()
        i += 1

    i = 0
    rospy.loginfo("Going backwards for 3 seconds")
    while not rospy.is_shutdown() and i < rate*run_time:
        ctrl_interface.send_control(0, -test_speed)
        rospy.loginfo_throttle(run_time, ctrl_interface)
        r.sleep()
        i += 1

    i = 0
    rospy.loginfo("Turning left for 3 seconds")
    while not rospy.is_shutdown() and i < rate*run_time:
        ctrl_interface.send_control(test_angle, 0.0)
        rospy.loginfo_throttle(run_time, ctrl_interface)
        r.sleep()
        i += 1
    i = 0

    rospy.loginfo("Turning right for 3 seconds")
    while not rospy.is_shutdown() and i < rate*run_time:
        ctrl_interface.send_control(-test_angle, 0.0)
        rospy.loginfo_throttle(run_time, ctrl_interface)
        r.sleep()
        i += 1

    rospy.loginfo("Going idle")
    rospy.spin()

if __name__ == '__main__': main()

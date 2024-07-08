#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped

import utm


class GPSToUTMRelay:
    def __init__(self) -> None:
        try:
            # Initialize node
            rospy.init_node('gps_to_utm_relay', anonymous=True)
            
            # Topic Parameters
            self.gps_topic = rospy.get_param('~gps_topic', 'gps/fix')
            
            # Publishers
            self.utm_topic = self.gps_topic + '/utm'
            self.utm_pub = rospy.Publisher(self.utm_topic, PointStamped, queue_size=10)
            
            # Subscribers
            self.gps_sub = rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_callback, queue_size=10)            
        
        except Exception as e:
            # Log error
            rospy.logerr(e)

        else:
            # Log status
            rospy.loginfo('{} node initialized.'.format(rospy.get_name()))
            
    def run(self) -> None:
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo('Shutting down {}'.format(rospy.get_name()))

    def gps_callback(self, msg):
        easting, northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
        point = PointStamped()
        point.header = msg.header
        point.point.x = easting
        point.point.y = northing
        point.point.z = msg.altitude
        self.utm_pub.publish(point)
        
        
if __name__ == '__main__':
    node = GPSToUTMRelay()
    node.run()
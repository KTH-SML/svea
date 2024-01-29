#!/usr/bin/env python3
import rospy
import tf2_ros

from pyproj import Proj

from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from sensor_msgs.msg import NavSatFix

####################################################
### Publish the reference gps frame in utm frame ### 
####################################################

class reference_gps_frame:
    def __init__(self):
        # Initialize node
        rospy.init_node('reference_gps_frame')

        # Reference GPS latitude, Longitude
        self.reference_gps = rospy.get_param("~reference_gps", [59.350791, 18.067825]) #ITRL

        # Publisher
        self.reference_gps_publisher = rospy.Publisher('/gps/reference_point', NavSatFix, queue_size=10)

        # Transformation
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.static_br = tf2_ros.StaticTransformBroadcaster()

        # Variables
        self.rate = rospy.Rate(10)
        self.seq = 0

    def PublishReferenceGpsFrame(self):
        projection = Proj(proj='utm', zone=34, ellps='WGS84')
        utm = projection(self.reference_gps[1], self.reference_gps[0])
        FixedGpsFrame = TransformStamped()
        FixedGpsFrame.header.stamp = rospy.Time.now()
        FixedGpsFrame.header.frame_id = "utm"
        FixedGpsFrame.child_frame_id = "reference_gps"
        FixedGpsFrame.transform.translation = Vector3(*[utm[0], utm[1], 0.0])
        FixedGpsFrame.transform.rotation = Quaternion(*[0.0, 0.0, 0.0, 1.0])
        self.static_br.sendTransform(FixedGpsFrame)

        reference_msg = NavSatFix()
        reference_msg.header.stamp = rospy.Time.now()
        reference_msg.header.frame_id = "utm"
        reference_msg.latitude = self.reference_gps[0]
        reference_msg.longitude = self.reference_gps[1]
        
        while not rospy.is_shutdown():
            reference_msg.header.seq = self.seq
            self.seq += 1
            self.reference_gps_publisher.publish(reference_msg)
            self.rate.sleep()

if __name__ == '__main__':
    reference_gps_frame().PublishReferenceGpsFrame()

#! /usr/bin/env python3

import rospy
import tf2_ros
import tf.transformations as tr
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

import math


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)

def replace_base(old, new) -> str:
    split_last = lambda xs: (xs[:-1], xs[-1])
    is_private = new.startswith('~')
    is_global = new.startswith('/')
    assert not (is_private or is_global)
    ns, _ = split_last(old.split('/'))
    ns += new.split('/')
    return '/'.join(ns)


class OdomToPathRelay:
        
        def __init__(self) -> None:
            try:
                # Initialize node
                rospy.init_node('odom_to_path_relay', anonymous=True)
                
                # Topic Parameters
                self.odom_topic = load_param('~odom_topic', 'odom')
                
                # Other parameters
                self.base_frame_id = load_param('~base_frame_id', 'base_link')
                
                # TF2
                self.tf_buf = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
                
                # Publishers
                self.path_topic = self.odom_topic + '/path'
                self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)
                
                # Subscribers
                self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
                
                # Initialize path
                self.path = Path()
            
            except Exception as e:
                # Log error
                rospy.logerr(e)

            else:
                # Sleep for 1 second for tf2 buffer to fill
                rospy.sleep(1)
                
                # Log status
                rospy.loginfo('{} node initialized.'.format(rospy.get_name()))
                
        def run(self) -> None:
            try:
                rospy.spin()
            except rospy.ROSInterruptException:
                rospy.loginfo('Shutting down {}'.format(rospy.get_name()))
        
        def odom_callback(self, msg: Odometry) -> None:
            try:
                # Update path header
                self.path.header = msg.header
                
                # Create pose stamped
                pose_stamped = PoseStamped()
                pose_stamped.header = msg.header
                pose_stamped.pose = msg.pose.pose
                
                # Transform pose to base frame
                if pose_stamped.header.frame_id != self.base_frame_id:
                    try:
                        pose_stamped = self.transform_to_base(pose_stamped)            
                    except Exception as e:
                        # Log error
                        rospy.logerr("{}: {}".format(rospy.get_name(), e))
                        return
                    
                # Append pose to path
                self.path.poses.append(pose_stamped)
                
                # Publish path
                self.path_pub.publish(self.path)
            except Exception as e:
                # Log error
                rospy.logerr(e)
                
        def transform_to_base(self, pose: PoseStamped) -> PoseStamped:
            # Convert pose to matrix form for transformation from map frame to pose frame
            matrix_mp_trans = tr.translation_matrix((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
            matrix_mp_rot = tr.quaternion_matrix((pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w))
            matrix_mp = tr.concatenate_matrices(matrix_mp_trans, matrix_mp_rot)            
            
            # Get matrix of transform from pose frame to base frame
            transform_pb = self.tf_buf.lookup_transform(self.base_frame_id, pose.header.frame_id, pose.header.stamp, rospy.Duration(1.0))
            matrix_pb_trans = tr.translation_matrix((transform_pb.transform.translation.x, transform_pb.transform.translation.y, transform_pb.transform.translation.z))
            matrix_pb_rot = tr.quaternion_matrix((transform_pb.transform.rotation.x, transform_pb.transform.rotation.y, transform_pb.transform.rotation.z, transform_pb.transform.rotation.w))
            matrix_pb = tr.concatenate_matrices(matrix_pb_trans, matrix_pb_rot)
            
            # Get matrix of transform from map frame to base frame
            matrix_mb = tr.concatenate_matrices(matrix_mp, matrix_pb)
            
            # Extract translation and quaternion from matrix
            translation = tr.translation_from_matrix(matrix_mb)
            quaternion = tr.quaternion_from_matrix(matrix_mb)
            
            # Convert to pose stamped and return
            pose_transformed = PoseStamped()
            pose_transformed.header = pose.header
            pose_transformed.pose.position.x = translation[0]
            pose_transformed.pose.position.y = translation[1]
            pose_transformed.pose.position.z = translation[2]
            pose_transformed.pose.orientation.x = quaternion[0]
            pose_transformed.pose.orientation.y = quaternion[1]
            pose_transformed.pose.orientation.z = quaternion[2]
            pose_transformed.pose.orientation.w = quaternion[3]
            
            return pose_transformed
        
        
if __name__ == '__main__':
    node = OdomToPathRelay()
    node.run()
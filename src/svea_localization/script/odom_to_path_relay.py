#! /usr/bin/env python3
"""
Author: Sulthan Suresh Fazeela
"""

<<<<<<<< HEAD:src/svea_localization/scripts/odom_to_path_relay.py
========
__author__ = "Sulthan Suresh Fazeela"
__email__ = "sultha@kth.se"
__license__ = "MIT"

>>>>>>>> 360ef06 (add svea_localization,modifying to ros2 in progress):src/svea_localization/script/odom_to_path_relay.py
import rclpy
import rclpy.exceptions
from rclpy.node import Node
import tf2_ros
import tf_transformations as tr
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

import math


<<<<<<< HEAD
<<<<<<<< HEAD:src/svea_localization/scripts/odom_to_path_relay.py
========
def load_param(self, name, value=None):
    self.declare_parameter(name, value)
    if value is None:
        assert self.has_parameter(name), f'Missing parameter "{name}"'
    return self.get_parameter(name).value
>>>>>>>> 360ef06 (add svea_localization,modifying to ros2 in progress):src/svea_localization/script/odom_to_path_relay.py
=======
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)

def replace_base(old, new) -> str:
    split_last = lambda xs: (xs[:-1], xs[-1])
    is_private = new.startswith('~')
    is_global = new.startswith('/')
    assert not (is_private or is_global)
    ns, _ = split_last(old.split('/'))
    ns += new.split('/')
    return '/'.join(ns)


class OdomToPathRelay(Node):
        
    def __init__(self) -> None:
        try:
            # Initialize node (No more Anonymous=True)
            super().__init__('odom_to_path_relay')
            
            # Topic Parameters
<<<<<<< HEAD
<<<<<<<< HEAD:src/svea_localization/scripts/odom_to_path_relay.py
            self.odom_topic = self.load_param('~odom_topic', 'odom')
            
            # Other parameters
            self.base_frame_id = self.load_param('~base_frame_id', 'base_link')
            self.initial_yaw_offset = self.load_param('~initial_yaw_offset', 0.0)
========
            self.odom_topic = load_param(self, '~odom_topic', 'odom')
            
            # Other parameters
            self.base_frame_id = load_param(self, '~base_frame_id', 'base_link')
            self.initial_yaw_offset = load_param(self, '~initial_yaw_offset', 0.0)
>>>>>>>> 360ef06 (add svea_localization,modifying to ros2 in progress):src/svea_localization/script/odom_to_path_relay.py
=======
            self.odom_topic = self.load_param('~odom_topic', 'odom')
            
            # Other parameters
            self.base_frame_id = self.load_param('~base_frame_id', 'base_link')
            self.initial_yaw_offset = self.load_param('~initial_yaw_offset', 0.0)
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)
            
            # TF2
            self.tf_buf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
            
            # Publishers
            self.path_topic = self.odom_topic + '/path'
            self.path_pub = self.create_publisher(Path, self.path_topic, 10)
            
            # Subscribers
            self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
            
            # Initialize path
            self.path = Path()
        
        except Exception as e:
            # Log error
            self.get_logger().error(str(e))

        else:
            # Sleep for 1 second for tf2 buffer to fill
            rclpy.spin_once(self, timeout_sec=1)
            
            # Log status
            self.get_logger().info('{} node initialized.'.format(self.get_name()))
<<<<<<< HEAD
<<<<<<<< HEAD:src/svea_localization/scripts/odom_to_path_relay.py
=======
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)
    
    def load_param(self, name, value=None):
        self.declare_parameter(name, value)
        if value is None:
            assert self.has_parameter(name), f'Missing parameter "{name}"'
        return self.get_parameter(name).value
    
<<<<<<< HEAD
========
            
>>>>>>>> 360ef06 (add svea_localization,modifying to ros2 in progress):src/svea_localization/script/odom_to_path_relay.py
=======
>>>>>>> e76035e (Added rmw-zenoh in dockerfile, added svea_example)
    def run(self) -> None:
        try:
            rclpy.spin(self)
        except rclpy.shutdown():
            self.get_logger().info('Shutting down {}'.format(self.get_name()))
    
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
                    self.get_logger().info('Shutting down {}'.format(self.get_name(), e))
                    return
                
            # Apply initial yaw offset to pose by rotating about z-axis
            if self.initial_yaw_offset != 0.0:
                pose_stamped = self.apply_initial_yaw_offset(pose_stamped)

            # Append pose to path
            self.path.poses.append(pose_stamped)

            # Publish path
            self.path_pub.publish(self.path)
        except Exception as e:
            # Log error
            self.get_logger().error(str(e))
            
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
    
    def apply_initial_yaw_offset(self, pose: PoseStamped) -> PoseStamped:
        # Convert current pose to matrix form
        position_matrix = tr.translation_matrix((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
        rotation_matrix = tr.quaternion_matrix((pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w))
        pose_matrix = tr.concatenate_matrices(position_matrix, rotation_matrix)
        
        # Convert yaw offset to rotation matrix
        yaw_offset_matrix = tr.rotation_matrix(self.initial_yaw_offset, (0, 0, 1))
        
        # Apply yaw offset to pose matrix
        updated_pose_matrix = tr.concatenate_matrices(yaw_offset_matrix, pose_matrix)
        
        # Extract translation and quaternion from updated pose matrix
        updated_position = tr.translation_from_matrix(updated_pose_matrix)
        updated_quaternion = tr.quaternion_from_matrix(updated_pose_matrix)
        
        # Convert to pose stamped and return
        updated_pose = PoseStamped()
        updated_pose.header = pose.header
        updated_pose.pose.position.x = updated_position[0]
        updated_pose.pose.position.y = updated_position[1]
        updated_pose.pose.position.z = updated_position[2]
        updated_pose.pose.orientation.x = updated_quaternion[0]
        updated_pose.pose.orientation.y = updated_quaternion[1]
        updated_pose.pose.orientation.z = updated_quaternion[2]
        updated_pose.pose.orientation.w = updated_quaternion[3]
        
        return updated_pose
        
def main():
    rclpy.init() 
    node = OdomToPathRelay()
    node.run()
        
if __name__ == '__main__':
    main()
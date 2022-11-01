#include <string>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <svea_msgs/VehicleState.h>
#include <svea_msgs/lli_ctrl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// Constants
ros::Duration tf_lookup_timeout;
// Parameters
std::string map_frame;
bool wait_for_transform;
bool publish_odometry;
bool publish_pose;
// Globals
tf2_ros::Buffer tfBuffer;
ros::Publisher state_pub;
ros::Publisher odom_pub;
ros::Publisher pose_pub;

svea_msgs::VehicleState stateFromOdom(const nav_msgs::Odometry& msg)
{
  double roll, pitch, yaw;
  tf2::Quaternion quat;
  tf2::fromMsg(msg.pose.pose.orientation, quat);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  svea_msgs::VehicleState state_msg;
  state_msg.header = msg.header;
  state_msg.child_frame_id = msg.child_frame_id;
  state_msg.x = msg.pose.pose.position.x;
  state_msg.y = msg.pose.pose.position.y;
  state_msg.yaw = yaw;
  state_msg.v = static_cast<float>(msg.twist.twist.linear.x);
  state_msg.covariance[0] = msg.pose.covariance[0];     // xx
  state_msg.covariance[1] = msg.pose.covariance[1];     // xy
  state_msg.covariance[4] = msg.pose.covariance[6];     // yy
  state_msg.covariance[5] = msg.pose.covariance[7];     // yx
  state_msg.covariance[10] = msg.pose.covariance[35];   // yaw yaw
  state_msg.covariance[15] = msg.twist.covariance[0];   // vv

  return state_msg;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // transform of "odom" to "map"
  geometry_msgs::TransformStamped odom_to_map;

  // if wait_for_transform
  // then get transform for when msg was sent
  // else get the latest transform
  ros::Time time = wait_for_transform ? msg->header.stamp : ros::Time(0);

  try {
    odom_to_map = tfBuffer.lookupTransform(map_frame, msg->header.frame_id, time, tf_lookup_timeout);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN_DELAYED_THROTTLE(2, "%s", ex.what());
    return;
  }

  geometry_msgs::PoseStamped odom_pose;
  odom_pose.pose = msg->pose.pose;
  odom_pose.header = msg->header;

  geometry_msgs::PoseStamped map_pose;
  tf2::doTransform<geometry_msgs::PoseStamped>(odom_pose, map_pose, odom_to_map);

  // copy odometry but update pose in map frame
  nav_msgs::Odometry odom = *msg;
  odom.header.frame_id = map_frame;
  odom.pose.pose = map_pose.pose;

  svea_msgs::VehicleState state = stateFromOdom(odom);
  state_pub.publish(state);

  if (publish_odometry) {
    odom_pub.publish(odom);
  }

  if (publish_pose) {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header = odom.header;
    pose_msg.pose = odom.pose;
    pose_pub.publish(pose_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_to_map");

  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  ros::TransportHints transportHints;
  transportHints.tcpNoDelay();

  tf_lookup_timeout = ros::Duration(0.1);

  // Get parameters
  map_frame = private_node.param<std::string>("map_frame", "map");
  wait_for_transform = private_node.param<bool>("wait_for_transform", false);
  publish_odometry = private_node.param<bool>("publish_odometry", true);
  publish_pose = private_node.param<bool>("publish_pose", true);

  // Create publishers
  state_pub = node.advertise<svea_msgs::VehicleState>("state", 2);
  odom_pub = node.advertise<nav_msgs::Odometry>("odometry/corrected", 2);
  pose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 2);

  tf2_ros::TransformListener tfListener(tfBuffer, true, transportHints);

  ros::Subscriber odom_sub = node.subscribe("odometry/filtered", 2, odomCallback, transportHints);

  ros::spin();

  return 0;
};


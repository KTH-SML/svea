#include <string>
#include <math.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <svea_msgs/lli_ctrl.h>
#include <svea_msgs/VehicleState.h>
#include "noetic_tf2_ros/noetic_transform_listener.h"

tf2_ros::Buffer tfBuffer;
ros::Publisher statePub;
ros::Publisher odomPub;
ros::Publisher posePub;
ros::Duration MAX_TF_WAIT;
std::string MAP_FRAME;
bool WAIT_FOR_TRANSFORM;
bool PUBLISH_ODOMETRY;
bool PUBLISH_POSE;

svea_msgs::VehicleState stateFromOdom(const nav_msgs::Odometry& odomMsg){
  svea_msgs::VehicleState stateMsg;
  stateMsg.header = odomMsg.header;
  stateMsg.child_frame_id = odomMsg.child_frame_id;
  stateMsg.x = odomMsg.pose.pose.position.x;
  stateMsg.y = odomMsg.pose.pose.position.y;
  double roll, pitch, yaw;
  tf2::Quaternion quat;
  tf2::fromMsg(odomMsg.pose.pose.orientation, quat);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  stateMsg.yaw = yaw;
  stateMsg.v = static_cast<float>(odomMsg.twist.twist.linear.x);
  stateMsg.covariance[0] = odomMsg.pose.covariance[0];  // xx
  stateMsg.covariance[1] = odomMsg.pose.covariance[1];  // xy
  stateMsg.covariance[4] = odomMsg.pose.covariance[6];  // yy
  stateMsg.covariance[5] = odomMsg.pose.covariance[7];  // yx
  stateMsg.covariance[10] = odomMsg.pose.covariance[35];  // yaw yaw
  stateMsg.covariance[15] = odomMsg.twist.covariance[0]; // vv
  return stateMsg;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  static unsigned int sequenceNumber = 0;
  const std::string& odomFrameId(msg->header.frame_id);
  geometry_msgs::TransformStamped odomToMapTf;
  ros::Time tfTime = WAIT_FOR_TRANSFORM ? msg->header.stamp : ros::Time(0);
  // if (WAIT_FOR_TRANSFORM) {
  //   tfTime = msg->header.stamp;
  // } else {
  //   tfTime = ros::Time(0);
  // }
  try {
    odomToMapTf = tfBuffer.lookupTransform(MAP_FRAME, odomFrameId, tfTime, MAX_TF_WAIT);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN_DELAYED_THROTTLE(2 ,"%s",ex.what());
    return;
  }
  geometry_msgs::PoseStamped odomPose;
  odomPose.pose = msg->pose.pose;
  odomPose.header = msg->header;
  geometry_msgs::PoseStamped mapPose;
  tf2::doTransform<geometry_msgs::PoseStamped>(odomPose, mapPose, odomToMapTf);
  nav_msgs::Odometry odomMsg;
  odomMsg.header.seq = sequenceNumber;
  odomMsg.header.stamp = msg->header.stamp;
  odomMsg.header.frame_id = MAP_FRAME;
  odomMsg.child_frame_id = msg->child_frame_id;
  odomMsg.pose.pose = mapPose.pose;
  odomMsg.pose.covariance = msg->pose.covariance;
  odomMsg.twist = msg->twist;
  svea_msgs::VehicleState stateMsg = stateFromOdom(odomMsg);
  statePub.publish(stateMsg);
  if (PUBLISH_ODOMETRY) {
    odomPub.publish(odomMsg);
  }
  if (PUBLISH_POSE) {
    geometry_msgs::PoseWithCovarianceStamped poseMsg;
    poseMsg.header = odomMsg.header;
    poseMsg.pose = odomMsg.pose;
    posePub.publish(poseMsg);
  }
  sequenceNumber++;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_to_map_node");
    
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std::string odometry_topic("odometry/filtered");
  std::string state_topic("state");
  std::string ctrl_topic("lli/ctrl_actuated");
  
  const std::string default_map_frame("map");
  MAP_FRAME = private_nh.param<std::string>("map_frame", default_map_frame);
  WAIT_FOR_TRANSFORM = private_nh.param<bool>("wait_for_transform", false);
  PUBLISH_ODOMETRY = private_nh.param<bool>("publish_odometry", true);
  PUBLISH_POSE = private_nh.param<bool>("publish_pose", true);
  
  MAX_TF_WAIT = ros::Duration(0.1);
  ros::TransportHints transportHints;
  transportHints.tcpNoDelay();
  tf2_ros::NoeticTransformListener tfListener(tfBuffer, true, transportHints);

  statePub = nh.advertise<svea_msgs::VehicleState>("state", 2);
  odomPub = nh.advertise<nav_msgs::Odometry>("odometry/corrected", 2);
  posePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 2);
  ros::Subscriber odomSub = nh.subscribe(odometry_topic, 2, odomCallback, transportHints);
  
  ros::spin();
  return 0;
};


/* Copyright (c) 2019-2021 Tobias Bolin
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
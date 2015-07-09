/**
 * @file      ptam_wrapper.cpp
 * @brief     Main class for Nao localization
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-07-07
 * @copyright (MIT) 2015 Edinferno
 */

#include "localization/ptam_wrapper.hpp"

PTAMWrapper::PTAMWrapper(ros::NodeHandle nh) {
  nh_ = nh;
  this->rosSetup();
}

PTAMWrapper::~PTAMWrapper() {;}

void PTAMWrapper::init() {
  outside_field_ = false;
}

void PTAMWrapper::rosSetup() {
  ROS_INFO_STREAM("Setting up Localization");
  ptam_pose_sub_ = nh_.subscribe("/vslam/robot_pose", 1, &PTAMWrapper::poseCB,
                                 this);
  ptam_info_sub_ = nh_.subscribe("/vslam/info", 1, &PTAMWrapper::infoCB,
                                 this);
  get_odom_pose_client_ = nh_.serviceClient<motion_msgs::GetRobotPosition>(
                            "/motion/get_robot_position", true);
  get_odom_pose_client_.waitForExistence();
  srv_set_pose_offset_ =
    nh_.advertiseService("set_pose_offset", &PTAMWrapper::setPoseOffset, this);
  srv_get_robot_pose_ =
    nh_.advertiseService("get_robot_pose", &PTAMWrapper::getRobotPose, this);
  get_robot_pos_srv_.request.use_sensors = true;
}

void PTAMWrapper::loadParams() {
  ros::param::param("field_length", field_length_, 9000.0f);
  ros::param::param("field_width", field_width_, 6000.0f);
}

void PTAMWrapper::poseCB(const
                         geometry_msgs::PoseWithCovarianceStamped::ConstPtr&
                         msg) {
  ptam_pose_ = *msg;
  get_odom_pose_client_.call(get_robot_pos_srv_);
  // Store odom pose given by robot, used for odometry offset
  last_odom_pose_ = get_robot_pos_srv_.response.position;
}

void PTAMWrapper::infoCB(const ptam_com::ptam_info::ConstPtr& msg) {
  ptam_info_ = *msg;
}

bool PTAMWrapper::setPoseOffset(localization_msgs::SetPoseOffset::Request& req,
                                localization_msgs::SetPoseOffset::Response& res) {
  // Transform PTAM's orientation in quaternion into Euler
  tf::Quaternion q;
  tf::quaternionMsgToTF(ptam_pose_.pose.pose.orientation, q);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Store pose offset robot pose is in the pitch frame of reference
  pose_offset_.x = -ptam_pose_.pose.pose.position.z - req.offset.x;
  pose_offset_.y = ptam_pose_.pose.pose.position.x - req.offset.y;
  pose_offset_.theta = pitch - req.offset.theta;
  return true;
}

bool PTAMWrapper::getRobotPose(localization_msgs::GetRobotPose::Request& req,
                               localization_msgs::GetRobotPose::Response& res) {
  // Get current pose from odometry
  get_odom_pose_client_.call(get_robot_pos_srv_);
  curr_odom_pose_ = get_robot_pos_srv_.response.position;

  // Transform PTAM's orientation quaternion into Euler
  tf::Quaternion q;
  tf::quaternionMsgToTF(ptam_pose_.pose.pose.orientation, q);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Calculate odometry offset by comparing current pose and ptam odom pose
  odom_diff_.x = curr_odom_pose_.x - last_odom_pose_.x;
  odom_diff_.y = curr_odom_pose_.y - last_odom_pose_.y;
  odom_diff_.theta = curr_odom_pose_.theta - last_odom_pose_.theta;

  // Update Robot pose given odometry offset and ptam
  curr_robot_pose_.x =
    -ptam_pose_.pose.pose.position.z - pose_offset_.x + odom_diff_.x;
  curr_robot_pose_.y =
    -ptam_pose_.pose.pose.position.x - pose_offset_.y + odom_diff_.y;
  curr_robot_pose_.theta =
    -pitch - pose_offset_.theta + odom_diff_.theta;

  // Check whether current pose is inside pitch
  if (abs(curr_robot_pose_.x) > field_length_ / 2) {
    outside_field_ = true;
  } else if (abs(curr_robot_pose_.y) > field_width_ / 2) {
    outside_field_ = true;
  } else {
    outside_field_ = false;
  }

  res.pose = curr_robot_pose_;
  return true;
}

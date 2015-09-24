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
  this->init();
}

PTAMWrapper::~PTAMWrapper() {;}

void PTAMWrapper::init() {
  outside_field_ = false;
  ptam_active_srv_.request.monitor_mode = MonitorMode::PTAM_ACTIVE;
  ptam_inactive_srv_.request.monitor_mode = MonitorMode::PTAM_LOST;
  ptam_rec_ = false;
}

void PTAMWrapper::rosSetup() {
  ROS_INFO_STREAM("Setting up Localization");
  ptam_pose_sub_ = nh_.subscribe("/vslam/robot_pose", 1, &PTAMWrapper::poseCB,
                                 this);
  // ptam_info_sub_ = nh_.subscribe("/vslam/info", 1, &PTAMWrapper::infoCB,
  //                                this);
  ros::service::waitForService("/motion/get_robot_position");
  get_odom_pose_client_ = nh_.serviceClient<motion_msgs::GetRobotPosition>(
                            "/motion/get_robot_position", true);
  ros::service::waitForService("/motion_planning/monitor_mode");
  monitor_client_ = nh_.serviceClient<motion_planning_msgs::MonitorMode>(
                      "/motion_planning/monitor_mode", true);
  robot_pose_pub_ = nh_.advertise <geometry_msgs::Pose2D>
                    ("/world/robot_pose", 1, true);
  srv_set_pose_offset_ =
    nh_.advertiseService("set_pose_offset", &PTAMWrapper::setPoseOffset, this);
  srv_get_robot_pose_ =
    nh_.advertiseService("get_robot_pose", &PTAMWrapper::getRobotPose, this);
  get_robot_pos_srv_.request.use_sensors = true;
}

void PTAMWrapper::update() {
  this->calcCurrPose();
  // if (ptam_rec_ > 0) {
  //   monitor_client_.call(ptam_active_srv_);
  // } else if (ptam_rec_ < 0) {
  //   monitor_client_.call(ptam_inactive_srv_);
  // }
  // ptam_rec_ -= 1;
}

void PTAMWrapper::loadParams() {
  ros::param::param("field_length", field_length_, 9.0f);
  ros::param::param("field_width", field_width_, 6.0f);
}

void PTAMWrapper::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // robot_pose_pub_.publish(curr_robot_pose_);
  ptam_pose_ = *msg;
  ptam_rec_ = 10;
  get_odom_pose_client_.call(get_robot_pos_srv_);
  // Store odom pose given by robot, used for odometry offset
  last_odom_pose_ = get_robot_pos_srv_.response.position;
}

// void PTAMWrapper::infoCB(const ptam_com::ptam_info::ConstPtr& msg) {
//   ptam_info_ = *msg;
// }

bool PTAMWrapper::setPoseOffset(localization_msgs::SetPoseOffset::Request& req,
                                localization_msgs::SetPoseOffset::Response& res) {
  // Transform PTAM's orientation in quaternion into Euler
  tf::Quaternion q;
  tf::quaternionMsgToTF(ptam_pose_.pose.orientation, q);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Store pose offset robot pose is in the pitch frame of reference
  pose_offset_.x = ptam_pose_.pose.position.x - req.offset.x;
  pose_offset_.y = ptam_pose_.pose.position.y - req.offset.y;
  pose_offset_.theta = yaw - req.offset.theta;
  return true;
}

bool PTAMWrapper::getRobotPose(localization_msgs::GetRobotPose::Request& req,
                               localization_msgs::GetRobotPose::Response& res) {
  this->calcCurrPose();
  res.pose = curr_robot_pose_;
  return true;
}

void PTAMWrapper::calcCurrPose() {
  // Get current pose from odometry
  get_odom_pose_client_.call(get_robot_pos_srv_);
  curr_odom_pose_ = get_robot_pos_srv_.response.position;

  // Transform PTAM's orientation quaternion into Euler
  tf::Quaternion q;
  tf::quaternionMsgToTF(ptam_pose_.pose.orientation, q);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Calculate odometry offset by comparing current pose and ptam odom pose
  odom_diff_.x = curr_odom_pose_.x - last_odom_pose_.x;
  odom_diff_.y = curr_odom_pose_.y - last_odom_pose_.y;
  odom_diff_.theta = curr_odom_pose_.theta - last_odom_pose_.theta;

  // Update Robot pose given odometry offset and ptam
  curr_robot_pose_.x =
    ptam_pose_.pose.position.x - pose_offset_.x + odom_diff_.x;
  curr_robot_pose_.y =
    ptam_pose_.pose.position.y - pose_offset_.y + odom_diff_.y;
  curr_robot_pose_.theta =
    yaw - pose_offset_.theta + odom_diff_.theta;

  // Check whether current pose is inside pitch
  if (abs(curr_robot_pose_.x) > field_length_ / 2) {
    outside_field_ = true;
  } else if (abs(curr_robot_pose_.y) > field_width_ / 2) {
    outside_field_ = true;
  } else {
    outside_field_ = false;
  }
  robot_pose_pub_.publish(curr_robot_pose_);
}

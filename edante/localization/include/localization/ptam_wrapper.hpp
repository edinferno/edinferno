/**
 * @file      ptam_wrapper.hpp
 * @brief     Main class for Nao localization
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-07-07
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef PTAM_WRAPPER_HPP
#define PTAM_WRAPPER_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>
#include <ptam_com/ptam_info.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose2D.h>
#include <motion_msgs/GetRobotPosition.h>
#include <motion_msgs/Position.h>
#include <signalling_msgs/signalling_values.hpp>
#include <motion_planning_msgs/MonitorMode.h>
#include <localization_msgs/GetRobotPose.h>
#include <localization_msgs/SetPoseOffset.h>

class PTAMWrapper {
 public:
  PTAMWrapper(ros::NodeHandle nh);

  ~PTAMWrapper();

  void init();

  void rosSetup();

  void update();

  void loadParams();

  void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void infoCB(const ptam_com::ptam_info::ConstPtr& msg);

  bool setPoseOffset(localization_msgs::SetPoseOffset::Request& req,
                     localization_msgs::SetPoseOffset::Response& res);

  bool getRobotPose(localization_msgs::GetRobotPose::Request& req,
                    localization_msgs::GetRobotPose::Response& res);

  void calcCurrPose();

 private:
  // Constants
  float field_length_;
  float field_width_;

  // Flags
  bool outside_field_;
  int ptam_rec_;

  ros::NodeHandle nh_;

  ros::Subscriber ptam_pose_sub_;
  ros::Subscriber ptam_info_sub_;
  ros::ServiceClient get_odom_pose_client_;
  ros::ServiceClient monitor_client_;
  ros::Publisher robot_pose_pub_;
  ros::ServiceServer srv_set_pose_offset_;
  ros::ServiceServer srv_get_robot_pose_;
  motion_msgs::GetRobotPosition get_robot_pos_srv_;
  motion_msgs::Position last_odom_pose_;
  motion_msgs::Position curr_odom_pose_;
  motion_planning_msgs::MonitorMode ptam_active_srv_;
  motion_planning_msgs::MonitorMode ptam_inactive_srv_;
  geometry_msgs::Pose2D odom_diff_;
  geometry_msgs::Pose2D pose_offset_;
  geometry_msgs::PoseStamped ptam_pose_;
  geometry_msgs::Pose2D curr_robot_pose_;
  ptam_com::ptam_info ptam_info_;
};

#endif /* PTAM_WRAPPER_HPP */

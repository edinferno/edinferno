#ifndef ALIGN_TO_POSE_HPP
#define ALIGN_TO_POSE_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// Messages
#include <std_srvs/Empty.h>
#include <motion_msgs/MoveTo.h>
#include <motion_msgs/MoveToward.h>
#include <navigation_msgs/AlignToPoseAction.h>
#include <localization_msgs/GetRobotPose.h>

class AlignToPoseAction {
 protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<navigation_msgs::AlignToPoseAction> as_;
  std::string action_name_;
  navigation_msgs::AlignToPoseFeedback feedback_;
  navigation_msgs::AlignToPoseResult result_;

 public:
  AlignToPoseAction(ros::NodeHandle nh, std::string name);

  ~AlignToPoseAction(void);

  void init();

  void rosSetup();

  void goalCB();

  void preemptCB();

  void executeCB();

 private:
  // Constants
  float x_scalar_;
  float y_scalar_;
  float dist_thresh_;
  float theta_scalar_;
  float theta_thresh_;

  // Variables
  float rel_target_theta_;

  // ROS
  geometry_msgs::Pose2D target_pose_;
  geometry_msgs::Pose2D rel_error_;
  ros::ServiceClient get_robot_pose_client_;
  localization_msgs::GetRobotPose curr_pose_;
  // motion_msgs::MoveTo move_to_srv_;
  ros::ServiceClient move_toward_client_;
  motion_msgs::MoveToward move_toward_srv_;
  std_srvs::Empty stop_move_srv_;
  ros::ServiceClient stop_move_client_;
  std_srvs::Empty move_init_srv_;
  ros::ServiceClient move_init_client_;
};


#endif /* ALIGN_TO_POSE_HPP */


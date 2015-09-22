#ifndef LOOK_AT_POSE_HPP
#define LOOK_AT_POSE_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose2D.h>
// Messages
#include <std_srvs/Empty.h>
#include <motion_msgs/SetAngles.h>
#include <motion_msgs/SetAngles.h>
#include <motion_msgs/AreResourcesAvailable.h>
#include <motion_msgs/IsEnabled.h>
#include <motion_msgs/TaskResource.h>
#include <navigation_msgs/LookAtPoseAction.h>
#include <vision_msgs/BallDetection.h>
#include <camera_msgs/SetActiveCamera.h>
#include <motion_planning_msgs/MonitorMode.h>
#include <localization_msgs/GetRobotPose.h>
// Navigation values
#include "motion_msgs/motion_values.hpp"
// Signalling values
#include "signalling_msgs/signalling_values.hpp"

class LookAtPoseAction {
 protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<navigation_msgs::LookAtPoseAction> as_;
  std::string action_name_;
  navigation_msgs::LookAtPoseFeedback feedback_;
  navigation_msgs::LookAtPoseResult result_;

 public:
  LookAtPoseAction(ros::NodeHandle nh, std::string name);

  ~LookAtPoseAction(void);

  void init();

  void ballCB(const vision_msgs::BallDetection::ConstPtr& msg);

  void goalCB();

  void preemptCB();

  void executeCB();

 private:
  // Constants

  // Variables

  // Flags
  bool going_;
  bool ball_found_;

  geometry_msgs::Pose2D curr_robot_pose_;
  geometry_msgs::Pose2D target_pose_;
  geometry_msgs::Pose2D rel_target_pose_;
  geometry_msgs::Pose2D torso_norm_pose_;
  ros::Subscriber ball_pos_sub_;
  ros::ServiceClient curr_pose_client_;
  localization_msgs::GetRobotPose curr_pose_srv_;
  ros::ServiceClient look_client_;
  motion_msgs::SetAngles look_pose_srv_;
  ros::ServiceClient stop_move_client_;
  std_srvs::Empty stop_move_srv_;
  ros::ServiceClient move_init_client_;
  std_srvs::Empty move_init_srv_;
  ros::ServiceClient resource_avail_client_;
  motion_msgs::AreResourcesAvailable resource_avail_srv_;
  ros::ServiceClient move_is_active_client_;
  motion_msgs::IsEnabled move_is_active_srv_;
  ros::ServiceClient kill_task_client_;
  motion_msgs::TaskResource kill_task_srv_;
  ros::ServiceClient camera_client_;
  camera_msgs::SetActiveCamera top_camera_srv_;
  ros::ServiceClient monitor_client_;
  motion_planning_msgs::MonitorMode top_camera_monitor_srv_;
  motion_planning_msgs::MonitorMode ball_seen_monitor_srv_;
  motion_planning_msgs::MonitorMode ball_lost_monitor_srv_;
};


#endif /* LOOK_AT_POSE_HPP */


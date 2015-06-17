#ifndef SEARCH_FOR_BALL_HPP
#define SEARCH_FOR_BALL_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// Messages
#include <std_srvs/Empty.h>
#include <motion_msgs/MoveTo.h>
#include <motion_msgs/SetAngles.h>
#include <motion_msgs/AngleInterp.h>
#include <motion_msgs/AreResourcesAvailable.h>
#include <motion_msgs/IsEnabled.h>
#include <motion_msgs/TaskResource.h>
#include <navigation_msgs/SearchForBallAction.h>
#include <vision_msgs/BallDetection.h>
#include <camera_msgs/SetActiveCamera.h>
#include <motion_planning_msgs/MonitorMode.h>
// Navigation values
#include "navigation/navigation_values.hpp"
// Signalling values
#include "signalling_msgs/signalling_values.hpp"

class SearchForBallAction {
 protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<navigation_msgs::SearchForBallAction> as_;
  std::string action_name_;
  navigation_msgs::SearchForBallFeedback feedback_;
  navigation_msgs::SearchForBallResult result_;

 public:
  SearchForBallAction(ros::NodeHandle nh, std::string name);

  ~SearchForBallAction(void);

  void init();

  void ballCB(const vision_msgs::BallDetection::ConstPtr& msg);

  void goalCB();

  void preemptCB();

  void scan_right();
  void scan_left();

  void executeCB();

 private:
  // Constants
  int WAIT_100MS_;
  int N_SCANS_;

  // Variables
  int scan_no_;

  // Flags
  bool going_;
  bool ball_found_;
  bool scanning_right_;
  bool scanning_left_;
  bool turning_;
  bool time_out_;

  ros::Subscriber ball_pos_sub_;
  motion_msgs::AngleInterp look_left_srv_;
  motion_msgs::AngleInterp look_straight_srv_;
  motion_msgs::AngleInterp scan_left_srv_;
  motion_msgs::AngleInterp scan_right_srv_;
  motion_msgs::MoveTo turn_left_srv_;
  motion_msgs::MoveTo turn_right_srv_;
  motion_msgs::AreResourcesAvailable resource_avail_srv_;
  motion_msgs::TaskResource kill_task_srv_;
  motion_msgs::IsEnabled move_is_active_srv_;
  camera_msgs::SetActiveCamera bottom_camera_srv_;
  camera_msgs::SetActiveCamera top_camera_srv_;
  ros::ServiceClient scan_client_;
  ros::ServiceClient turn_client_;
  std_srvs::Empty stop_move_srv_;
  ros::ServiceClient stop_move_client_;
  std_srvs::Empty move_init_srv_;
  ros::ServiceClient move_init_client_;
  ros::ServiceClient resource_avail_client_;
  ros::ServiceClient move_is_active_client_;
  ros::ServiceClient kill_task_client_;
  ros::ServiceClient camera_client_;
  ros::ServiceClient monitor_client_;
  motion_planning_msgs::MonitorMode bottom_camera_monitor_srv_;
  motion_planning_msgs::MonitorMode top_camera_monitor_srv_;
  motion_planning_msgs::MonitorMode ball_seen_monitor_srv_;
};


#endif /* SEARCH_FOR_BALL_HPP */


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
// Navigation values
#include "navigation/navigation_values.hpp"

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

  void goalCB(const vision_msgs::BallDetection::ConstPtr& msg);

  void scan();

  void executeCB(const navigation_msgs::SearchForBallGoalConstPtr& goal);

 private:
  // Constants
  int WAIT_100MS_;
  int N_SCANS_;

  // Variables
  int scan_no_;

  // Flags
  bool ball_found_;
  bool scanning_;
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
  ros::ServiceClient scan_client_;
  ros::ServiceClient turn_client_;
  std_srvs::Empty stop_move_srv_;
  ros::ServiceClient stop_move_client_;
  std_srvs::Empty move_init_srv_;
  ros::ServiceClient move_init_client_;
  ros::ServiceClient resource_avail_client_;
  ros::ServiceClient move_is_active_client_;
  ros::ServiceClient kill_task_client_;
};


#endif /* SEARCH_FOR_BALL_HPP */


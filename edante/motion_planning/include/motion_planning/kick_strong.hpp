/**
 * @file      kick_strong.hpp
 * @brief     Strong kicking action with 4.2 second execution
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-07
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef KICK_STRONG_HPP
#define KICK_STRONG_HPP

// System
#include <cmath>
#include <string>

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// Messages
#include <std_srvs/Empty.h>
#include <motion_planning_msgs/KickStrongAction.h>
#include <motion_msgs/PositionInterpolation.h>
#include <motion_msgs/GoToBalance.h>
#include <motion_msgs/SetPosture.h>
#include <motion_msgs/FootState.h>
#include <motion_msgs/Enable.h>
#include <motion_msgs/SetPosture.h>
#include <motion_msgs/motion_values.hpp>

class KickStrongAction {
 protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_planning_msgs::KickStrongAction> as_;
  std::string action_name_;
  motion_planning_msgs::KickStrongFeedback feedback_;
  motion_planning_msgs::KickStrongResult result_;

 public:
  KickStrongAction(ros::NodeHandle nh, std::string name);

  ~KickStrongAction(void);

  void init();

  void goalCB();

  void preemptCB();

  void executeCB();

  void setKickType();

  void setupLiftLeg();
  void setupFootBack();
  void setupKickForw();
  void setupRetractFoot();
  void setupLowerLeg();

 private:
  // Flags
  bool going;

  // Values
  float balance_init_;
  float kick_time_;
  float move_time_;
  float lift_time_;
  float leg_lift_ ;
  float foot_retraction_;
  float foot_forward_kick_;
  float foot_lift_;
  float foot_rot_kick_;
  float balance_end_;

  // Variables
  uint8_t kick_type_;

  // ROS
  ros::ServiceClient move_init_client_;
  ros::ServiceClient balance_client_;
  ros::ServiceClient pos_interp_client_;
  ros::ServiceClient foot_client_;
  ros::ServiceClient goto_balance_client_;

  motion_msgs::Enable enable_balance_;
  motion_msgs::Enable disable_balance_;
  motion_msgs::FootState left_foot_state_;
  motion_msgs::FootState right_foot_state_;
  motion_msgs::GoToBalance goto_balance_init_;
  motion_msgs::GoToBalance goto_balance_final_;
  motion_msgs::PositionInterpolation lift_leg_;
  motion_msgs::PositionInterpolation foot_back_;
  motion_msgs::PositionInterpolation kick_forw_;
  motion_msgs::PositionInterpolation retract_foot_;
  motion_msgs::PositionInterpolation lower_leg_;
};


#endif /* KICK_STRONG_HPP */

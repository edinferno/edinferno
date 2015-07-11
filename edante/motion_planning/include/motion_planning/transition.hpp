/**
 * @file      transition.hpp
 * @brief     Transition action server
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-07
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef TRANSITION_HPP
#define TRANSITION_HPP

// System
#include <cmath>

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// Messages
#include <std_srvs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
// #include <comms_msgs/GameState.h>
#include <motion_msgs/SetPosture.h>
#include <motion_planning_msgs/TransitionAction.h>
#include <signalling_msgs/signalling_values.hpp>
#include <spl_msgs/RoboCupGameControlData.h>

class TransitionAction {
 protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_planning_msgs::TransitionAction> as_;
  std::string action_name_;
  motion_planning_msgs::TransitionFeedback feedback_;
  motion_planning_msgs::TransitionResult result_;

 public:
  TransitionAction(ros::NodeHandle nh, std::string name);

  ~TransitionAction(void);

  void goalCB();

  void preemptCB();

  void init();

  void checkChestTransition(const std_msgs::UInt8::ConstPtr& msg);
  void checkGCTransition(const std_msgs::UInt8::ConstPtr& msg);
  void checkFallenTransition(const std_msgs::Bool::ConstPtr& msg);
  void checkPenalizedTransition(const std_msgs::UInt8::ConstPtr& msg);
  void checkSmachOnline(const std_msgs::Bool::ConstPtr& msg);

 private:
  // Flags
  bool smach_online_;
  bool disabled_;

  // Variables
  uint8_t state_;
  uint8_t game_state_;
  uint8_t chest_presses_;

  // ROS
  std_srvs::Empty wakeup_srv_;
  motion_msgs::SetPosture stand_srv_;
  ros::ServiceClient wakeup_client_;
  ros::ServiceClient stand_client_;
  std_srvs::Empty rest_srv_;
  ros::ServiceClient rest_client_;
  ros::Subscriber chest_sub_;
  ros::Subscriber game_state_sub_;
  ros::Subscriber has_fallen_sub_;
  ros::Subscriber penalized_sub_;
  ros::Subscriber smach_online_sub_;
  std_srvs::Empty stop_move_srv_;
  ros::ServiceClient stop_move_client_;
  ros::Publisher manual_penalized_pub_;
};


#endif /* TRANSITION_HPP */

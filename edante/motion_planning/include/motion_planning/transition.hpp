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
#include <motion_planning_msgs/TransitionAction.h>
#include <signalling_msgs/signalling_values.hpp>

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

  void init();

  void goalCB();

  void preemptCB();

  void checkTransition(const std_msgs::UInt8::ConstPtr& msg);

 private:
  // Flags

  // Variables
  uint8_t state_;
  uint8_t chest_presses_;

  // ROS
  ros::Subscriber chest_sub_;
  std_srvs::Empty stop_move_srv_;
  ros::ServiceClient stop_move_client_;
};


#endif /* TRANSITION_HPP */

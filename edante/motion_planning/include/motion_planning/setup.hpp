/**
 * @file      setup.hpp
 * @brief     Setup action server
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-07
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef SETUP_HPP
#define SETUP_HPP

// System
#include <cmath>

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// Messages
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <motion_planning_msgs/SetupAction.h>
#include <signalling_msgs/FadeRGB.h>
// #include <signalling_msgs/signalling_values.hpp>

class SetupAction {
 protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_planning_msgs::SetupAction> as_;
  std::string action_name_;
  motion_planning_msgs::SetupFeedback feedback_;
  motion_planning_msgs::SetupResult result_;

 public:
  SetupAction(ros::NodeHandle nh, std::string name);

  ~SetupAction(void);

  void init();

  void goalCB();

  void preemptCB();

  void executeCB();

 private:
  // Flags

  // Variables
  uint8_t state_;

  // Constants
  static const int BLACK = 0x000000;
  static const int BLUE = 0x0000FF;
  static const int GREEN = 0x00FF00;
  static const int YELLOW = 0xFFFF00;
  static const int RED = 0xFF0000;
  static const int WHITE = 0xFFFFFF;

  static const uint INITIAL = 0;
  static const uint READY = 1;
  static const uint SET = 2;
  static const uint PENALIZED = 3;
  static const uint PLAYING = 4;
  static const uint FINISHED = 5;

  // ROS
  signalling_msgs::FadeRGB initial_rgb_srv_;
  signalling_msgs::FadeRGB ready_rgb_srv_;
  signalling_msgs::FadeRGB set_rgb_srv_;
  signalling_msgs::FadeRGB penalized_rgb_srv_;
  signalling_msgs::FadeRGB playing_rgb_srv_;
  signalling_msgs::FadeRGB finished_rgb_srv_;
  ros::ServiceClient fade_rgb_client_;
  std_srvs::Empty stop_move_srv_;
  ros::ServiceClient stop_move_client_;
};


#endif /* SETUP_HPP */

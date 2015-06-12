/**
 * @file      stand_up.hpp
 * @brief     Stand up action server
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-07
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef STAND_UP_HPP
#define STAND_UP_HPP

// System
#include <cmath>

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// Messages
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <motion_planning_msgs/StandUpAction.h>
#include <motion_msgs/GetPostureFamily.h>
#include <motion_msgs/SetPosture.h>

class StandUpAction {
 protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_planning_msgs::StandUpAction> as_;
  std::string action_name_;
  motion_planning_msgs::StandUpFeedback feedback_;
  motion_planning_msgs::StandUpResult result_;

 public:
  StandUpAction(ros::NodeHandle nh, std::string name);

  ~StandUpAction(void);

  void init();

  void goalCB();

  void preemptCB();

  void awakeCB(const std_msgs::Bool::ConstPtr& msg);

  void executeCB();

 private:
  // Flags
  bool is_awake_;

  // ROS
  ros::Subscriber awake_sub_;
  std_srvs::Empty wake_up_srv_;
  ros::ServiceClient wake_up_client_;
  motion_msgs::GetPostureFamily get_posture_family_srv_;
  ros::ServiceClient get_posture_family_client_;
  std_srvs::Empty stop_move_srv_;
  ros::ServiceClient stop_move_client_;
  motion_msgs::SetPosture set_posture_srv_;
  ros::ServiceClient set_posture_client_;

};


#endif /* STAND_UP_HPP */

/**
 * @file      sit_rest.hpp
 * @brief     Sit down action server
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-07
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef SIT_REST_HPP
#define SIT_REST_HPP

// System
#include <cmath>

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// Messages
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <motion_planning_msgs/SitRestAction.h>
#include <motion_msgs/GetPostureFamily.h>
#include <motion_msgs/SetPosture.h>

class SitRestAction {
 protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_planning_msgs::SitRestAction> as_;
  std::string action_name_;
  motion_planning_msgs::SitRestFeedback feedback_;
  motion_planning_msgs::SitRestResult result_;

 public:
  SitRestAction(ros::NodeHandle nh, std::string name);

  ~SitRestAction(void);

  void init();

  void awakeCB(const std_msgs::Bool::ConstPtr& msg);

  void executeCB(const motion_planning_msgs::SitRestGoalConstPtr& goal);

 private:
  // Flags
  bool is_awake_;

  // ROS
  ros::Subscriber awake_sub_;
  std_srvs::Empty rest_srv_;
  ros::ServiceClient rest_client_;
  std_srvs::Empty stop_move_srv_;
  ros::ServiceClient stop_move_client_;
};


#endif /* SIT_REST_HPP */

/**
 * @file      look_at_ball.cpp
 * @brief     Nao stands up and tracks ball with head
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef LOOK_AT_BALL_HPP
#define LOOK_AT_BALL_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// Messages
// #include <std_srvs/Empty.h>
#include <motion_msgs/AngleInterp.h>
#include <vision_msgs/StartHeadTracking.h>
#include <vision_msgs/StopHeadTracking.h>
#include <vision_msgs/BallDetection.h>
#include <navigation_msgs/LookAtBallAction.h>


class LookAtBallAction {
 protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<navigation_msgs::LookAtBallAction> as_;
  std::string action_name_;
  navigation_msgs::LookAtBallFeedback feedback_;
  navigation_msgs::LookAtBallResult result_;

 public:
  LookAtBallAction(ros::NodeHandle nh, std::string name);

  ~LookAtBallAction(void);

  void init();

  void rosSetup();

  void ballCB(const vision_msgs::BallDetection::ConstPtr& msg);

  void goalCB();

  void preemptCB();

  void executeCB();

 private:
  // Flags
  bool ball_found_;
  bool tracking_;

  // Variables

  // ROS
  ros::Subscriber ball_pos_sub_;
  ros::ServiceClient start_head_track_client_;
  ros::ServiceClient stop_head_track_client_;
  ros::ServiceClient look_client_;
  motion_msgs::AngleInterp look_straight_srv_;
  vision_msgs::StartHeadTracking start_head_track_srv_;
  vision_msgs::StopHeadTracking stop_head_track_srv_;
};


#endif /* LOOK_AT_BALL_HPP */


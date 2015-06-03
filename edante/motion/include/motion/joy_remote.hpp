/**
 * @file      joy_remote.h
 * @brief     Joystick remote control node
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-22
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef JOY_REMOTE_HPP
#define JOY_REMOTE_HPP

// System
#include <cmath>

// ROS
#include <ros/ros.h>

// Messages
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <motion_msgs/MoveToward.h>
#include <motion_msgs/SetPosture.h>
#include <motion_msgs/ChangeAngles.h>

class JoyRemote {
 public:
  explicit JoyRemote(ros::NodeHandle* nh);

  ~JoyRemote();

  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  void sendCmd();

  void wakeUp();

  void moveInit();

  void stand();

  void rest();

 private:
  // Flags
  bool wakeUpFlag;
  bool moveInitFlag;
  bool standFlag;
  bool restFlag;

  // ROS
  ros::NodeHandle* nh_;
  ros::Subscriber joy_sub_;
  motion_msgs::MoveToward moveSrv;
  std_srvs::Empty wakeUpSrv;
  std_srvs::Empty moveInitSrv;
  motion_msgs::SetPosture standSrv;
  std_srvs::Empty restSrv;
  std_srvs::Empty stopSrv;
  motion_msgs::ChangeAngles headSrv;
  ros::ServiceClient moveTowardClient;
  ros::ServiceClient wakeUpClient;
  ros::ServiceClient moveInitClient;
  ros::ServiceClient standClient;
  ros::ServiceClient restClient;
  ros::ServiceClient stopMoveClient;
  ros::ServiceClient changeAnglesClient;
};

#endif  /* JOY_REMOTE_HPP */

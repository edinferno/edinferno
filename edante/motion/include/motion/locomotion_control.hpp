/**
 * @file      locomotion_control.hpp
 * @brief     ROS wrapper for NaoQI locomotion control methods
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef LOCOMOTION_CONTROL_HPP
#define LOCOMOTION_CONTROL_HPP

// System
#include <vector>

// NaoQi
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <althread/almutex.h>

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// Messages
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <motion_msgs/Move.h>
#include <motion_msgs/MoveTo.h>
#include <motion_msgs/MoveToward.h>
#include <motion_msgs/GetMoveConfig.h>
#include <motion_msgs/GetRobotPosition.h>
#include <motion_msgs/GetNextRobotPosition.h>
#include <motion_msgs/GetRobotVelocity.h>
#include <motion_msgs/GetWalkArmsEnabled.h>
#include <motion_msgs/SetWalkArmsEnabled.h>
#include <motion_msgs/IsEnabled.h>

using std::vector;

class LocomotionControl : public AL::ALModule  {
 public:
  LocomotionControl(boost::shared_ptr<AL::ALBroker> broker,
                    const std::string& name);
  ~LocomotionControl();

  void init();

  void rosSetup(ros::NodeHandle* nh);

  // ROS services
  bool move(motion_msgs::Move::Request& req,
            motion_msgs::Move::Response& res);

  bool moveTo(motion_msgs::MoveTo::Request& req,
              motion_msgs::MoveTo::Response& res);
  bool moveToward(motion_msgs::MoveToward::Request& req,
                  motion_msgs::MoveToward::Response& res);

  bool moveInit(std_srvs::Empty::Request& req,
                std_srvs::Empty::Response& res);
  bool waitUntilMoveIsFinished(std_srvs::Empty::Request& req,
                               std_srvs::Empty::Response& res);
  bool stopMove(std_srvs::Empty::Request& req,
                std_srvs::Empty::Response& res);
  bool getMoveConfig(motion_msgs::GetMoveConfig::Request& req,
                     motion_msgs::GetMoveConfig::Response& res);
  bool getRobotPosition(motion_msgs::GetRobotPosition::Request& req,
                        motion_msgs::GetRobotPosition::Response& res);
  bool getNextRobotPosition(motion_msgs::GetNextRobotPosition::Request& req,
                            motion_msgs::GetNextRobotPosition::Response& res);
  bool getRobotVelocity(motion_msgs::GetRobotVelocity::Request& req,
                        motion_msgs::GetRobotVelocity::Response& res);
  bool getWalkArmsEnabled(motion_msgs::GetWalkArmsEnabled::Request& req,
                          motion_msgs::GetWalkArmsEnabled::Response& res);
  bool setWalkArmsEnabled(motion_msgs::SetWalkArmsEnabled::Request& req,
                          motion_msgs::SetWalkArmsEnabled::Response& res);
  bool moveIsActive(motion_msgs::IsEnabled::Request& req,
                    motion_msgs::IsEnabled::Response& res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher moving_pub_;
  ros::ServiceServer srv_move_;
  ros::ServiceServer srv_move_to_;
  ros::ServiceServer srv_move_toward_;
  ros::ServiceServer srv_move_init_;
  ros::ServiceServer srv_wait_move_finished_;
  ros::ServiceServer srv_stop_move_;
  ros::ServiceServer srv_get_move_config_;
  ros::ServiceServer srv_get_robot_position_;
  ros::ServiceServer srv_get_next_robot_position_;
  ros::ServiceServer srv_get_robot_velocity_;
  ros::ServiceServer srv_get_walk_arms_enabled_;
  ros::ServiceServer srv_set_walk_arms_enabled_;
  ros::ServiceServer srv_move_is_active_;

  // NAOqi
  boost::shared_ptr<AL::ALMutex> fCallbackMutex;
  AL::ALMemoryProxy fMemoryProxy;
  AL::ALMotionProxy mProxy_;
};

#endif /* LOCOMOTION_CONTROL_HPP */

/**
 * @file      locomotion_control.h
 * @brief     ROS wrapper for NaoQI locomotion control methods
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef LOCOMOTION_CONTROL_H_
#define LOCOMOTION_CONTROL_H_

#include <ros/ros.h>
#include <motion/Move.h>
#include <motion/MoveTo.h>
#include <motion/MoveToward.h>
#include <motion/GetMoveConfig.h>
#include <motion/GetRobotPosition.h>
#include <motion/GetNextRobotPosition.h>
#include <motion/GetRobotVelocity.h>
#include <motion/GetWalkArmsEnabled.h>
#include <motion/SetWalkArmsEnabled.h>

#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <althread/almutex.h>

#include "definitions.h"

using std::vector;

class LocomotionControl : public AL::ALModule  {
 public:
  LocomotionControl(boost::shared_ptr<AL::ALBroker> broker,
                    const std::string& name);
  ~LocomotionControl();

  void init();

  void rosSetup(ros::NodeHandle* nh);

  // ROS services
  bool move(motion::Move::Request &req,
            motion::Move::Response &res);

  bool moveTo(motion::MoveTo::Request &req,
              motion::MoveTo::Response &res);
  bool moveToward(motion::MoveToward::Request &req,
                  motion::MoveToward::Response &res);

  bool moveInit(std_srvs::Empty::Request &req,
                std_srvs::Empty::Response &res);
  bool waitUntilMoveIsFinished(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res);
  bool stopMove(std_srvs::Empty::Request &req,
                std_srvs::Empty::Response &res);
  bool getMoveConfig(motion::GetMoveConfig::Request &req,
                     motion::GetMoveConfig::Response &res);
  bool getRobotPosition(motion::GetRobotPosition::Request &req,
                        motion::GetRobotPosition::Response &res);
  bool getNextRobotPosition(motion::GetNextRobotPosition::Request &req,
                            motion::GetNextRobotPosition::Response &res);
  bool getRobotVelocity(motion::GetRobotVelocity::Request &req,
                        motion::GetRobotVelocity::Response &res);
  bool getWalkArmsEnabled(motion::GetWalkArmsEnabled::Request &req,
                          motion::GetWalkArmsEnabled::Response &res);
  bool setWalkArmsEnabled(motion::SetWalkArmsEnabled::Request &req,
                          motion::SetWalkArmsEnabled::Response &res);

  // ROS publisher
  void checkMoveActive();

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
  std_msgs::Bool move_active;


  // NAOqi
  boost::shared_ptr<AL::ALMutex> fCallbackMutex;
  AL::ALMemoryProxy fMemoryProxy;
  AL::ALMotionProxy mProxy_;
};

#endif /* LOCOMOTION_CONTROL_H_ */

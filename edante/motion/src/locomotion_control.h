/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-18
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI locomotion control methods
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

#include <alproxies/almotionproxy.h>

#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <vector>

#include "definitions.h"

using std::vector;

class LocomotionControl {
 public:
  LocomotionControl(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy);
  ~LocomotionControl();

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
  bool moveIsActive();
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
  void spinTopics();

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


  // NAOqi
  AL::ALMotionProxy* mProxy_;
};

#endif /* LOCOMOTION_CONTROL_H_ */

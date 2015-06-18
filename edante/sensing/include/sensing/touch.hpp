/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-11
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI Fall Manager API
*/

#ifndef TOUCH_HPP
#define TOUCH_HPP

// System
#include <string>

// NaoQi
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <althread/almutex.h>

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// Messages
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <sensing_msgs/Bumpers.h>
#include <sensing_msgs/Hand.h>
#include <sensing_msgs/Head.h>

class Touch : public AL::ALModule {
 public:
  Touch(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
  ~Touch();

  void init();

  void rosSetup(ros::NodeHandle* nh);

  void rightBumperPressed();
  void leftBumperPressed();
  void frontHeadPressed();
  void middleHeadPressed();
  void rearHeadPressed();
  void handRightBackPressed();
  void handRightLeftPressed();
  void handRightRightPressed();
  void handLeftBackPressed();
  void handLeftLeftPressed();
  void handLeftRightPressed();
  void chestButtonPressed();
  void timerCB(const ros::TimerEvent& event);

 private:
  AL::ALMemoryProxy fMemoryProxy;

  boost::shared_ptr<AL::ALMutex> fCallbackMutex;

  float fState;
  size_t chest_presses_;

  // ROS
  ros::Timer timer;
  ros::NodeHandle* nh_;
  ros::Publisher bumpers_pub_;
  ros::Publisher chest_pub_;
  ros::Publisher head_pub_;
  ros::Publisher right_hand_pub_;
  ros::Publisher left_hand_pub_;
  sensing_msgs::Bumpers bumperMsg_;
  std_msgs::UInt8 chestMsg_;
  sensing_msgs::Head headMsg_;
  sensing_msgs::Hand rightHandMsg_;
  sensing_msgs::Hand leftHandMsg_;

  // NaoQI
  AL::ALMemoryProxy* memProxy_;
};

#endif /* TOUCH_H_ */

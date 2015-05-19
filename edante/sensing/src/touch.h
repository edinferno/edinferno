/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-11
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI Fall Manager API
*/

#ifndef TOUCH_H
#define TOUCH_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <althread/almutex.h>

#include <string>
#include "sensing/Bumpers.h"
#include "sensing/Hand.h"
#include "sensing/Head.h"
#include "definitions.h"

class Touch : public AL::ALModule {
 public:
  Touch(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
  ~Touch();

  void init();

  void rosSetup(ros::NodeHandle* nh);

  void rightBumperPressed();
  void leftBumperPressed();
  void singleChestClick();
  void doubleChestClick();
  void tripleChestClick();
  void frontHeadPressed();
  void middleHeadPressed();
  void rearHeadPressed();
  void handRightBackPressed();
  void handRightLeftPressed();
  void handRightRightPressed();
  void handLeftBackPressed();
  void handLeftLeftPressed();
  void handLeftRightPressed();

 private:
  AL::ALMemoryProxy fMemoryProxy;

  boost::shared_ptr<AL::ALMutex> fCallbackMutex;

  float fState;

  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher bumpers_pub_;
  ros::Publisher chest_pub_;
  ros::Publisher head_pub_;
  ros::Publisher right_hand_pub_;
  ros::Publisher left_hand_pub_;
  sensing::Bumpers bumperMsg_;
  std_msgs::String chestMsg_;
  sensing::Head headMsg_;
  sensing::Hand rightHandMsg_;
  sensing::Hand leftHandMsg_;

  // NaoQI
  AL::ALMemoryProxy* memProxy_;
};

#endif /* TOUCH_H_ */

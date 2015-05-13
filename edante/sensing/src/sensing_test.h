/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-11
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI Fall Manager API
*/

#ifndef SENSING_TEST_H
#define SENSING_TEST_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "sensing/bumpers.h"
#include "sensing/hand.h"
#include "sensing/head.h"

#include <alcommon/almodule.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>

#include "definitions.h"

#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <string>

#include <alproxies/almemoryproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <althread/almutex.h>

namespace AL {
class ALBroker;
}

using namespace std;

class Sensing_Test : public AL::ALModule {

 public:

  Sensing_Test(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
  virtual ~Sensing_Test();

  /** Overloading ALModule::init().
  * This is called right after the module has been loaded
  */
  virtual void init();

  void rightBumperPressed();
  void leftBumperPressed();
  void chestButtonPressed();
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
  sensing::bumpers bumperMsg_;
  sensing::head headMsg_;
  sensing::hand rightHandMsg_;
  sensing::hand leftHandMsg_;

  // NaoQI
  AL::ALMemoryProxy* memProxy_;
};

#endif /* SENSING_TEST_H_ */

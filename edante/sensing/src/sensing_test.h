/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-11
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI Fall Manager API
*/

#ifndef SENSING_H
#define SENSING_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>

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

class Sensing : public AL::ALModule {

 public:

  Sensing(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
  virtual ~Sensing();

  /** Overloading ALModule::init().
  * This is called right after the module has been loaded
  */
  virtual void init();

  /**
  * This method will be called every time the event RightBumperPressed is raised.
  */
  void onRightBumperPressed();

 private:
  // Flags
  // bool hasFallen_;
  AL::ALMemoryProxy fMemoryProxy;

  boost::shared_ptr<AL::ALMutex> fCallbackMutex;

  float fState;

  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher robot_fallen_pub_;

  // NaoQI
  AL::ALMemoryProxy* memProxy_;
};

#endif /* SENSING_H_ */

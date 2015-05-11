/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-11
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI Fall Manager API
*/

#ifndef FALL_MANAGER_H
#define FALL_MANAGER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <alcommon/almodule.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>

#include "motion/enable.h"
#include "motion/isEnabled.h"

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

class Fall_Manager : public AL::ALModule {

 public:
  // Fall_Manager(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy);
  // ~Fall_Manager();

  // void hasFallen();

  Fall_Manager(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
  virtual ~Fall_Manager();

  /** Overloading ALModule::init().
  * This is called right after the module has been loaded
  */
  virtual void init();

  /**
  * This method will be called every time the event RightBumperPressed is raised.
  */
  void onRightBumperPressed();

  // ROS services
  bool setFallManagerEnabled(motion::enable::Request &req,
                             motion::enable::Response &res);
  bool getFallManagerEnabled(motion::isEnabled::Request &req,
                             motion::isEnabled::Response &res);

 private:
  // Flags
  // bool hasFallen_;
  AL::ALMemoryProxy fMemoryProxy;

  boost::shared_ptr<AL::ALMutex> fCallbackMutex;

  float fState;

  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher robot_fallen_pub_;
  ros::ServiceServer srv_set_fall_manager_;
  ros::ServiceServer srv_get_fall_manager_;

  // NaoQI
  AL::ALMotionProxy* mProxy_;
  AL::ALMemoryProxy* memProxy_;
};

#endif /* FALL_MANAGER_H_ */

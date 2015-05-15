/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's power sensors
*/
#ifndef POWER_H
#define POWER_H

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include "sensing/sonars.h"
#include "sensing/enable.h"

#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alsonarproxy.h>
#include <althread/almutex.h>

#include "definitions.h"

namespace AL {
class ALBroker;
}
using namespace std;

class Power : public AL::ALModule  {
 public:
  Power(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
  ~Power();

  void init();
  // ROS publishers
  void rosSetup(ros::NodeHandle* nh);

  // void spin();

  void powerPub();

 private:
  // Flags
  std_msgs::String powerEvent;
  std_msgs::UInt32 powerCharge;

  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher power_status_pub_;
  ros::Publisher power_charge_pub_;

  // NaoQI
  boost::shared_ptr<AL::ALMutex> fCallbackMutex;
  AL::ALMemoryProxy fMemoryProxy;
};

#endif  /* POWER_H */

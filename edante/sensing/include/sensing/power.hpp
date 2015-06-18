/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's power sensors
*/
#ifndef POWER_HPP
#define POWER_HPP

// System
#include <string>

// NaoQi
#include <althread/almutex.h>
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alsonarproxy.h>

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// Messages
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensing_msgs/Enable.h>

class Power : public AL::ALModule  {
 public:
  Power(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
  ~Power();

  void init();
  // ROS publishers
  void rosSetup(ros::NodeHandle* nh);

  // void spin();

  void powerPub();

  void hotJointDetected();

 private:
  // Flags
  std_msgs::String powerEvent;
  std_msgs::Int32 powerCharge;

  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher power_status_pub_;
  ros::Publisher power_charge_pub_;

  // NaoQI
  boost::shared_ptr<AL::ALMutex> fCallbackMutex;
  AL::ALMemoryProxy fMemoryProxy;
};

#endif  /* POWER_H */

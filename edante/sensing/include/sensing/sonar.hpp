/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's sonar sensors
*/

#ifndef SONAR_HPP
#define SONAR_HPP

// System
#include <string>

// NaoQi
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alsonarproxy.h>
#include <althread/almutex.h>

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// Messages
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "sensing_msgs/Sonars.h"
#include "sensing_msgs/Enable.h"

class Sonar : public AL::ALModule {
 public:
  // ROS publishers
  Sonar(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
  ~Sonar();

  void init();

  void rosSetup(ros::NodeHandle* nh, bool pubSonars);
  void rosSetup(ros::NodeHandle* nh) {this->rosSetup(nh, false);}

  void spin();

  void sonarLeftDetected();
  void sonarRightDetected();
  void sonarLeftNothingDetected();
  void sonarRightNothingDetected();

  void pubSonars();

  bool enableSonarPub(sensing_msgs::Enable::Request& req,
                      sensing_msgs::Enable::Response& res);

 private:
  // Private
  bool pubSonars_;
  AL::ALMemoryProxy fMemoryProxy;

  boost::shared_ptr<AL::ALMutex> fCallbackMutex;

  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher sonar_event_pub_;
  ros::Publisher sonar_data_pub_;
  ros::ServiceServer srv_enab_sonar_;

  // NaoQI
  AL::ALSonarProxy* sonarProxy_;
};

#endif  /* SONAR_H */

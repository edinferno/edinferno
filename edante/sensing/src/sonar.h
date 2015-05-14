/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's sonar sensors
*/

#ifndef SONAR_H
#define SONAR_H

#include <ros/ros.h>
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "sensing/sonars.h"

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

class Sonar : public AL::ALModule {
 public:

  // ROS publishers
  Sonar(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
  ~Sonar();

  void init();

  void rosSetup(ros::NodeHandle* nh, bool pubSonars);
  void rosSetup(ros::NodeHandle* nh) {this->rosSetup(nh, true);}

  void spin();

  void sonarLeftDetected();
  void sonarRightDetected();
  void sonarLeftNothingDetected();
  void sonarRightNothingDetected();

  void pubSonars();

 private:
  // Private
  bool pubSonars_;
  AL::ALMemoryProxy fMemoryProxy;

  boost::shared_ptr<AL::ALMutex> fCallbackMutex;

  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher sonar_event_pub_;
  ros::Publisher sonar_data_pub_;

  // NaoQI
  AL::ALSonarProxy* sonarProxy_;
};

#endif  /* SONAR_H */

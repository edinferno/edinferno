/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's fsr sensors
*/
#ifndef FSR_H
#define FSR_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <boost/shared_ptr.hpp>

#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alsonarproxy.h>
#include <althread/almutex.h>

#include <sensing/Fsr.h>
#include "sensing/Enable.h"
#include "definitions.h"

namespace AL {
class ALBroker;
}

class Fsr : public AL::ALModule {
 public:
  Fsr(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
  ~Fsr();

  void init();

  void rosSetup(ros::NodeHandle* nh, bool pubFsrs);
  void rosSetup(ros::NodeHandle* nh) {this->rosSetup(nh, false);}

  void spin();

  // ROS publishers
  void pubContact();
  void pubFsr();

  bool enableFsrPub(sensing::Enable::Request &req,
                    sensing::Enable::Response &res);

 private:
  // Flags
  bool pubFsrs_;

  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher fsr_contact_pub_;
  ros::Publisher fsr_data_pub_;
  ros::ServiceServer srv_enab_fsr_;

  // NaoQI
  boost::shared_ptr<AL::ALMutex> fCallbackMutex;
  AL::ALMemoryProxy fMemoryProxy;
};

#endif  /* FSR_H */

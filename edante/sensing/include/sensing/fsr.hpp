/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's fsr sensors
*/
#ifndef FSR_H
#define FSR_H

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
#include <sensing_msgs/Fsr.h>
#include <sensing_msgs/Enable.h>

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

  bool enableFsrPub(sensing_msgs::Enable::Request& req,
                    sensing_msgs::Enable::Response& res);

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

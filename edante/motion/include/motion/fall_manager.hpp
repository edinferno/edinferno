/**
 * @file      fall_manager.h
 * @brief     ROS wrapper for NaoQI Fall Manager API
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef FALL_MANAGER_HPP
#define FALL_MANAGER_HPP
// NaoQi
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <althread/almutex.h>

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// Messages
#include <std_msgs/Bool.h>
#include <motion_msgs/Enable.h>
#include <motion_msgs/IsEnabled.h>

class FallManager : public AL::ALModule  {
 public:
  FallManager(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
  ~FallManager();

  void init();

  void rosSetup(ros::NodeHandle* nh);

  // ROS services
  bool setFallManagerEnabled(motion_msgs::Enable::Request& req,
                             motion_msgs::Enable::Response& res);
  bool getFallManagerEnabled(motion_msgs::IsEnabled::Request& req,
                             motion_msgs::IsEnabled::Response& res);

  void hasFallen();

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher has_fallen_pub_;
  ros::ServiceServer srv_set_fall_manager_;
  ros::ServiceServer srv_get_fall_manager_;
  std_msgs::Bool fallen_;

  // NaoQI
  boost::shared_ptr<AL::ALMutex> fCallbackMutex;
  AL::ALMotionProxy mProxy_;
  AL::ALMemoryProxy fMemoryProxy;
};

#endif /* FALL_MANAGER_HPP */

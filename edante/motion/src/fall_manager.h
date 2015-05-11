/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-11
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI Fall Manager API
*/

#ifndef FALL_MANAGER_H
#define FALL_MANAGER_H

#include <ros/ros.h>

#include <alproxies/almotionproxy.h>

#include "motion/enable.h"
#include "motion/isEnabled.h"

#include "definitions.h"

using namespace std;

class Fall_Manager {

 public:
  Fall_Manager(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy);
  ~Fall_Manager();

  // ROS services
  bool setFallManagerEnabled(motion::enable::Request &req,
                             motion::enable::Response &res);
  bool getFallManagerEnabled(motion::isEnabled::Request &req,
                             motion::isEnabled::Response &res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_set_fall_manager_;
  ros::ServiceServer srv_get_fall_manager_;

  // NaoQI
  AL::ALMotionProxy* mProxy_;
};

#endif /* FALL_MANAGER_H_ */

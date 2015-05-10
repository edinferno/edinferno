/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-10
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI Whole Body Balancer
*/

#ifndef BODY_BALANCER_H_
#define BODY_BALANCER_H_

#include <ros/ros.h>

#include <alproxies/almotionproxy.h>

#include "definitions.h"

using namespace std;

class Body_Balancer {

 public:
  Body_Balancer(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy);
  ~Body_Balancer();

 private:
// ROS
  ros::NodeHandle* nh_;

// NaoQI
  AL::ALMotionProxy* mProxy_;
};

#endif /* BODY_BALANCER_H_ */

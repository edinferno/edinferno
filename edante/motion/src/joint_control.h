/*
* @File: motion.h
* @Author: Alejandro Bordallo
* @Date:   2015-04-04 20:47:59
* @Last Modified by:   Alejandro Bordallo
* @Last Modified time: 2015-04-04 20:47:59
* @Desc: Defines the Joint Control Wrapper functions
*/

#ifndef JOINT_CONTROL_H_
#define JOINT_CONTROL_H_

#include <ros/ros.h>

#include "motion/handName.h"

#include <alproxies/almotionproxy.h>
#include <alerror/alerror.h>

#include "definitions.h"

using namespace std;

class Joint_Control{
public:
 Joint_Control();
 ~Joint_Control();

 // Joint control API

 // ROS publishers

 // ROS services
 bool closeHand(motion::handName::Request &req, motion::handName::Response &res);
 bool openHand(motion::handName::Request &req, motion::handName::Response &res);

private:
 // ROS
 ros::NodeHandle* nh_;
 ros::ServiceServer srv_close_hand_;
 ros::ServiceServer srv_open_hand_;

 // NAOqi
 AL::ALMotionProxy* mProxy_;

 // Internal

};
#endif /* JOINT_CONTROL_H_ */
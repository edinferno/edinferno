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

#include "motion/setAngles.h"
#include "motion/changeAngles.h"
#include "motion/getAngles.h"
#include "motion/useHand.h"

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
 bool setAngles(motion::setAngles::Request &req, motion::setAngles::Response &res);
 bool changeAngles(motion::changeAngles::Request &req, motion::changeAngles::Response &res);
 bool getAngles(motion::getAngles::Request &req, motion::getAngles::Response &res);
 bool closeHand(motion::useHand::Request &req, motion::useHand::Response &res);
 bool openHand(motion::useHand::Request &req, motion::useHand::Response &res);

private:
 // ROS
 ros::NodeHandle* nh_;
 ros::ServiceServer srv_set_angles_;
 ros::ServiceServer srv_change_angles_;
 ros::ServiceServer srv_get_angles_;
 ros::ServiceServer srv_close_hand_;
 ros::ServiceServer srv_open_hand_;

 // NAOqi
 AL::ALMotionProxy* mProxy_;

 // Internal

};
#endif /* JOINT_CONTROL_H_ */
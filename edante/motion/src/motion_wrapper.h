/*
* @File: motion_wrapper.h
* @Author: Alejandro Bordallo
* @Date:   2015-04-04 20:47:59
* @Last Modified by:   Alejandro Bordallo
* @Last Modified time: 2015-04-04 20:47:59
* @Desc: Defines the Motion Wrapper functions
*/

#ifndef MOTION_WRAPPER_H_
#define MOTION_WRAPPER_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
// #include <std_srvs/Trigger.h>
#include <vector>

#include "motion/setStiffness.h"
#include "motion/getStiffness.h"

#include <alproxies/almotionproxy.h>
#include <alerror/alerror.h>

#include "definitions.h"

using namespace std;

class Motion{
public:
 Motion();
 ~Motion();

 // Stiffness control API
 bool wakeUp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
 bool rest(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

 bool setStiffnesses(string& name, float& stiffness);
 bool setStiffnesses(string& name, const vector<float>& stiffnesses);
 bool setStiffnesses(const vector<string>& names, float& stiffness);
 bool setStiffnesses(const vector<string>& names,
  const vector<float>& stiffnesses);

 vector<float> getStiffnesses(const vector<string>&);

 // ROS publisher
 void spinTopics();

 // ROS services
 bool recStiffness(motion::setStiffness::Request &req,
  motion::setStiffness::Response &res);
 bool getStiffness(motion::getStiffness::Request &req,
  motion::getStiffness::Response &res);

private:
 // ROS
 ros::NodeHandle* nh_;
 ros::Publisher wake_pub_;
 ros::ServiceServer set_stiffness_;
 ros::ServiceServer get_stiffness_;
 ros::ServiceServer srv_awake_;
 ros::ServiceServer srv_rest_;

 // NAOqi
 AL::ALMotionProxy* mProxy_;

 // Internal
 bool awake_;
};

#endif /* MOTION_WRAPPER_H_ */

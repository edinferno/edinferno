#ifndef MOTION_TASK_H_
#define MOTION_TASK_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <vector>

#include <alproxies/almotionproxy.h>

#include "definitions.h"

using namespace std;

class Motion_Task{
public:
 Motion_Task();
 ~Motion_Task();

 //Motion task API
 bool killMove(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
 bool killAll(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);


private:
 //ROS
 ros::NodeHandle *nh_;
 ros::ServiceServer srv_kill_move_;
 ros::ServiceServer srv_kill_all_;

 // NAOqi
 AL::ALMotionProxy* mProxy_;

};

#endif /* MOTION_TASK_H_ */
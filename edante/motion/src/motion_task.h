#ifndef MOTION_TASK_H_
#define MOTION_TASK_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <vector>

#include "motion/task.h"

#include "motion/getTaskList.h"
#include "motion/taskResource.h"

#include <alproxies/almotionproxy.h>
#include <alerror/alerror.h>

#include "definitions.h"

using namespace std;

class Motion_Task{
public:
 Motion_Task(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy);
 ~Motion_Task();

 //Motion task API
 bool getTaskList(motion::getTaskList::Request &req,
                  motion::getTaskList::Response &res);
 bool killTasksResources(motion::taskResource::Request &req,
                         motion::taskResource::Response &res);
 bool killMove(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
 bool killAll(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

private:
 //ROS
 ros::NodeHandle *nh_;
 ros::ServiceServer srv_get_task_list_;
 ros::ServiceServer srv_kill_tasks_res_;
 ros::ServiceServer srv_kill_move_;
 ros::ServiceServer srv_kill_all_;

 // NAOqi
 AL::ALMotionProxy* mProxy_;

};

#endif /* MOTION_TASK_H_ */
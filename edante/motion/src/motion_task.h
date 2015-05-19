/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI motion task methods
*/

#ifndef MOTION_TASK_H_
#define MOTION_TASK_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <alproxies/almotionproxy.h>
#include <alerror/alerror.h>

#include <vector>
#include "motion/Task.h"
#include "motion/GetTaskList.h"
#include "motion/AreResourcesAvailable.h"
#include "motion/TaskResource.h"
#include "definitions.h"

class MotionTask {
 public:
  MotionTask(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy);
  ~MotionTask();

  // ROS services
  bool getTaskList(motion::GetTaskList::Request &req,
                   motion::GetTaskList::Response &res);
  bool areResourcesAvailable(motion::AreResourcesAvailable::Request &req,
                             motion::AreResourcesAvailable::Response &res);
  bool killTasksResources(motion::TaskResource::Request &req,
                          motion::TaskResource::Response &res);
  bool killMove(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool killAll(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

 private:
  // ROS
  ros::NodeHandle *nh_;
  ros::ServiceServer srv_get_task_list_;
  ros::ServiceServer srv_are_res_avail_;
  ros::ServiceServer srv_kill_tasks_res_;
  ros::ServiceServer srv_kill_move_;
  ros::ServiceServer srv_kill_all_;

  // NAOqi
  AL::ALMotionProxy* mProxy_;
};

#endif /* MOTION_TASK_H_ */

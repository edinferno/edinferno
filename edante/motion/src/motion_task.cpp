#include "motion_task.h"

Motion_Task::Motion_Task(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy) {
  nh_ = nh;
  mProxy_ = mProxy;
  INFO("Setting up Motion Task services" << std::endl);
  srv_get_task_list_ = nh_->advertiseService("getTaskList",
                       &Motion_Task::getTaskList, this);
  srv_are_res_avail_ = nh_->advertiseService("areResourcesAvailable",
                       &Motion_Task::areResourcesAvailable, this);
  srv_kill_tasks_res_ = nh_->advertiseService("killTasksUsingResources",
                        &Motion_Task::killTasksResources, this);
  srv_kill_move_ = nh_->advertiseService("killMove",
                                         &Motion_Task::killMove, this);
  srv_kill_all_ = nh_->advertiseService("killAll",
                                        &Motion_Task::killAll, this);
}

Motion_Task::~Motion_Task() {
  ros::shutdown();
}

bool Motion_Task::getTaskList(motion::getTaskList::Request &req,
                              motion::getTaskList::Response &res) {
  AL::ALValue taskList = mProxy_->getTaskList();
  string tName = taskList[0][0];
  int tID = taskList[0][1];
  DEBUG(tName << tID << std::endl);
  res.taskList.resize(2);
  res.taskList[0].taskName = tName;
  res.taskList[0].motionID = tID;
  return true;
}

bool Motion_Task::areResourcesAvailable(motion::areResourcesAvailable::Request
                                        &req,
                                        motion::areResourcesAvailable::Response &res) {
  res.available = mProxy_->areResourcesAvailable(req.resourceNames);
  return true;
}

bool Motion_Task::killTasksResources(motion::taskResource::Request &req,
                                     motion::taskResource::Response &res) {
  try {
    mProxy_->killTasksUsingResources(req.resourceNames);
    res.res = true;
  } catch (const std::exception& e) {
    ERR(e.what());
    res.res = false;
  }
  return true;
}

bool Motion_Task::killMove(std_srvs::Empty::Request &req,
                           std_srvs::Empty::Response &res) {
  try {
    mProxy_->killMove();
  } catch (const std::exception& e) {
    ERR(e.what());
  }
  return true;
}

bool Motion_Task::killAll(std_srvs::Empty::Request &req,
                          std_srvs::Empty::Response &res) {
  try {
    mProxy_->killAll();
  } catch (const std::exception& e) {
    ERR(e.what());
  }
  return true;
}

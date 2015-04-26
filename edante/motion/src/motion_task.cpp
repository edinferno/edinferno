#include "motion_task.h"

Motion_Task::Motion_Task()
{
	nh_ = new ros::NodeHandle();
  mProxy_ = new AL::ALMotionProxy("127.0.0.1", 9559);
	INFO("Setting up Motion Task services" << std::endl);
  srv_get_task_list_ = nh_->advertiseService("motion/getTaskList",
                                             &Motion_Task::getTaskList, this);
  srv_kill_tasks_res_ = nh_->advertiseService("motion/killTasksUsingResources", 
                                        &Motion_Task::killTasksResources, this);
	srv_kill_move_ = nh_->advertiseService("motion/killMove", 
                                        &Motion_Task::killMove, this);
  srv_kill_all_ = nh_->advertiseService("motion/killAll", 
                              &Motion_Task::killAll, this);
  
}

Motion_Task::~Motion_Task()
{
  ros::shutdown();  
}

bool Motion_Task::getTaskList(motion::getTaskList::Request &req,
                              motion::getTaskList::Response &res)
{
  res.taskList = mProxy_->getTaskList();
  return true;
}

bool Motion_Task::killTasksResources(motion::taskResource::Request &req,
                                     motion::taskResource::Response &res)
{
  try{
    mProxy_->killTasksUsingResources(req.resourceNames);
    res.res = true;
  }
  catch (const std::exception& e){
    ERR(e.what());
    res.res = false;
  }
  return true;
}

bool Motion_Task::killMove(std_srvs::Empty::Request &req,
                           std_srvs::Empty::Response &res)
{
  try{
    mProxy_->killMove();
  }
  catch (const std::exception& e){
    ERR(e.what());
  }
  return true;
}

bool Motion_Task::killAll(std_srvs::Empty::Request &req,
                          std_srvs::Empty::Response &res)
{
  try{
    mProxy_->killAll();
  }
  catch (const std::exception& e){
    ERR(e.what());
  }
  return true;
}

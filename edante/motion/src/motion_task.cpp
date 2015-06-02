/**
 * @file      motion_task.cpp
 * @brief     ROS wrapper for NaoQI motion task methods
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion_task.h"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <althread/alcriticalsection.h>

#include <qi/log.hpp>

MotionTask::MotionTask(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name): AL::ALModule(broker, name),
  fCallbackMutex(AL::ALMutex::createALMutex()) {
  qi::log::setVerbosity(qi::log::info);
  // setModuleDescription("Motion Task module.");
}

MotionTask::~MotionTask() {
}

void MotionTask::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());
    mProxy_ = AL::ALMotionProxy(getParentBroker());
  } catch (const AL::ALError& e) {
    DEBUG(e.what() << std::endl);
  }
}

void MotionTask::rosSetup(ros::NodeHandle* nh) {
  nh_ = nh;
  INFO("Setting up Motion Task services" << std::endl);
  srv_get_task_list_ = nh_->advertiseService("get_task_list",
                       &MotionTask::getTaskList, this);
  srv_are_res_avail_ = nh_->advertiseService("are_resources_available",
                       &MotionTask::areResourcesAvailable, this);
  srv_kill_tasks_res_ = nh_->advertiseService("kill_tasks_using_resources",
                        &MotionTask::killTasksResources, this);
  srv_kill_move_ = nh_->advertiseService("kill_move",
                                         &MotionTask::killMove, this);
  srv_kill_all_ = nh_->advertiseService("kill_all",
                                        &MotionTask::killAll, this);
}

bool MotionTask::getTaskList(motion::GetTaskList::Request &req,
                             motion::GetTaskList::Response &res) {
  AL::ALValue task_list = mProxy_.getTaskList();
  size_t s = task_list.getSize();
  res.task_list.resize(s);
  if (s > 0) {
    for (size_t i = 0; i < s; ++i) {
      res.task_list[i].task_name = static_cast<std::string>(task_list[i][0]);
      res.task_list[i].motion_id = static_cast<int>(task_list[i][1]);
    }
  }
  return true;
}

bool MotionTask::areResourcesAvailable(
  motion::AreResourcesAvailable::Request &req,
  motion::AreResourcesAvailable::Response &res) {
  res.available = mProxy_.areResourcesAvailable(req.resource_names);
  return true;
}

bool MotionTask::killTasksResources(motion::TaskResource::Request &req,
                                    motion::TaskResource::Response &res) {
  try {
    mProxy_.killTasksUsingResources(req.resource_names);
    res.res = true;
  } catch (const std::exception& e) {
    ERR(e.what());
    res.res = false;
  }
  return true;
}

bool MotionTask::killMove(std_srvs::Empty::Request &req,
                          std_srvs::Empty::Response &res) {
  try {
    mProxy_.killMove();
  } catch (const std::exception& e) {
    ERR(e.what());
  }
  return true;
}

bool MotionTask::killAll(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res) {
  try {
    mProxy_.killAll();
  } catch (const std::exception& e) {
    ERR(e.what());
  }
  return true;
}

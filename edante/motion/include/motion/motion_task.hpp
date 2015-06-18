/**
 * @file      motion_task.hpp
 * @brief     ROS wrapper for NaoQI motion task methods
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef MOTION_TASK_HPP
#define MOTION_TASK_HPP

// System
#include <vector>

// NaoQi
#include <alerror/alerror.h>
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <althread/almutex.h>

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// Messages
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <motion_msgs/Task.h>
#include <motion_msgs/GetTaskList.h>
#include <motion_msgs/AreResourcesAvailable.h>
#include <motion_msgs/TaskResource.h>

class MotionTask : public AL::ALModule  {
 public:
  MotionTask(boost::shared_ptr<AL::ALBroker> broker,
             const std::string& name);
  ~MotionTask();

  void init();

  void rosSetup(ros::NodeHandle* nh);

  // ROS services
  bool getTaskList(motion_msgs::GetTaskList::Request& req,
                   motion_msgs::GetTaskList::Response& res);
  bool areResourcesAvailable(motion_msgs::AreResourcesAvailable::Request& req,
                             motion_msgs::AreResourcesAvailable::Response& res);
  bool killTasksResources(motion_msgs::TaskResource::Request& req,
                          motion_msgs::TaskResource::Response& res);
  bool killMove(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool killAll(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_get_task_list_;
  ros::ServiceServer srv_are_res_avail_;
  ros::ServiceServer srv_kill_tasks_res_;
  ros::ServiceServer srv_kill_move_;
  ros::ServiceServer srv_kill_all_;

  // NAOqi
  boost::shared_ptr<AL::ALMutex> fCallbackMutex;
  AL::ALMemoryProxy fMemoryProxy;
  AL::ALMotionProxy mProxy_;
};

#endif /* MOTION_TASK_HPP */

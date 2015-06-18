/**
 * @file      robot_posture.h
 * @brief     ROS wrapper for NaoQI ALRobotPosture
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef ROBOT_POSTURE_H_
#define ROBOT_POSTURE_H_

// NaoQi
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <althread/almutex.h>
// Boost
#include <boost/shared_ptr.hpp>
// ROS
#include <ros/ros.h>
// Messages
#include <std_srvs/Empty.h>
#include <motion_msgs/GetPostureList.h>
#include <motion_msgs/SetPosture.h>
#include <motion_msgs/GetPostureFamily.h>
#include <motion_msgs/SetMaxTryNumber.h>

class RobotPosture : public AL::ALModule  {
 public:
  RobotPosture(boost::shared_ptr<AL::ALBroker> broker,
               const std::string& name);
  ~RobotPosture();

  void init();

  void rosSetup(ros::NodeHandle* nh);

  // ROS services
  bool getPostureList(motion_msgs::GetPostureList::Request& req,
                      motion_msgs::GetPostureList::Response& res);
  bool goToPosture(motion_msgs::SetPosture::Request& req,
                   motion_msgs::SetPosture::Response& res);
  bool applyPosture(motion_msgs::SetPosture::Request& req,
                    motion_msgs::SetPosture::Response& res);
  bool stopPosture(std_srvs::Empty::Request& req,
                   std_srvs::Empty::Response& res);
  bool getPostureFamily(motion_msgs::GetPostureFamily::Request& req,
                        motion_msgs::GetPostureFamily::Response& res);
  bool getPostureFamilyList(motion_msgs::GetPostureList::Request& req,
                            motion_msgs::GetPostureList::Response& res);
  bool setMaxTryNumber(motion_msgs::SetMaxTryNumber::Request& req,
                       motion_msgs::SetMaxTryNumber::Response& res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_get_posture_list_;
  ros::ServiceServer srv_go_to_posture_;
  ros::ServiceServer srv_apply_posture_;
  ros::ServiceServer srv_stop_posture_;
  ros::ServiceServer srv_get_posture_family_;
  ros::ServiceServer srv_get_posture_family_list_;
  ros::ServiceServer srv_set_max_try_number_;

  // NaoQI
  boost::shared_ptr<AL::ALMutex> fCallbackMutex;
  AL::ALMemoryProxy fMemoryProxy;
  AL::ALRobotPostureProxy pProxy_;
};

#endif /* ROBOT_POSTURE_H_ */

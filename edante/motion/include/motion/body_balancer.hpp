/**
 * @file      body_balancer.hpp
 * @brief     ROS wrapper for NaoQI Whole Body Balancer
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef BODY_BALANCER_HPP
#define BODY_BALANCER_HPP

// NaoQi
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
#include <motion_msgs/Enable.h>
#include <motion_msgs/FootState.h>
#include <motion_msgs/EnableBalanceConstraint.h>
#include <motion_msgs/GoToBalance.h>
#include <motion_msgs/EnableEffector.h>
#include <motion_msgs/SetEffectorControl.h>

class BodyBalancer : public AL::ALModule  {
 public:
  BodyBalancer(boost::shared_ptr<AL::ALBroker> broker,
               const std::string& name);
  ~BodyBalancer();

  void init();

  void rosSetup(ros::NodeHandle* nh);


  // ROS services
  bool enableBalance(motion_msgs::Enable::Request& req,
                     motion_msgs::Enable::Response& res);
  bool footState(motion_msgs::FootState::Request& req,
                 motion_msgs::FootState::Response& res);
  bool enableBalanceConstraint(
    motion_msgs::EnableBalanceConstraint::Request& req,
    motion_msgs::EnableBalanceConstraint::Response& res);
  bool goToBalance(motion_msgs::GoToBalance::Request& req,
                   motion_msgs::GoToBalance::Response& res);
  bool enableEffectorControl(motion_msgs::EnableEffector::Request& req,
                             motion_msgs::EnableEffector::Response& res);
  bool setEffectorControl(motion_msgs::SetEffectorControl::Request& req,
                          motion_msgs::SetEffectorControl::Response& res);
  bool enableEffectorOptimization(motion_msgs::EnableEffector::Request& req,
                                  motion_msgs::EnableEffector::Response& res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher balance_pub_;
  ros::ServiceServer srv_enable_balance_;
  ros::ServiceServer srv_foot_state_;
  ros::ServiceServer srv_enable_balance_constraint_;
  ros::ServiceServer srv_go_to_balance_;
  ros::ServiceServer srv_enable_effector_control_;
  ros::ServiceServer srv_set_effector_control_;
  ros::ServiceServer srv_enable_effector_optimization_;
  std_msgs::Bool isBalancing_;

  // NaoQI
  boost::shared_ptr<AL::ALMutex> fCallbackMutex;
  AL::ALMemoryProxy fMemoryProxy;
  AL::ALMotionProxy mProxy_;
};
#endif /* BODY_BALANCER_HPP */

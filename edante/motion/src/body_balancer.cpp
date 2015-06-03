/**
 * @file      body_balancer.cpp
 * @brief     ROS wrapper for NaoQI Whole Body Balancer
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-21
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion/body_balancer.hpp"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <althread/alcriticalsection.h>

#include <qi/log.hpp>

BodyBalancer::BodyBalancer(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name): AL::ALModule(broker, name),
  fCallbackMutex(AL::ALMutex::createALMutex()) {
  qi::log::setVerbosity(qi::log::info);
  // setModuleDescription("Balancer Test module.");
}

BodyBalancer::~BodyBalancer() {
}

void BodyBalancer::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());
    mProxy_ = AL::ALMotionProxy(getParentBroker());
  } catch (const AL::ALError& e) {
    ROS_ERROR_STREAM(e.what());
  }
}

void BodyBalancer::rosSetup(ros::NodeHandle* nh) {
  nh_ = nh;
  ROS_INFO_STREAM("Setting up Body Balancer publishers");
  balance_pub_ = nh_->advertise<std_msgs::Bool>("is_balancing", 10, true);

  ROS_INFO_STREAM("Setting up Body Balancer services");
  srv_enable_balance_ =
    nh_->advertiseService("enable_balance",
                          &BodyBalancer::enableBalance, this);
  srv_foot_state_ =
    nh_->advertiseService("foot_state",
                          &BodyBalancer::footState, this);
  srv_enable_balance_constraint_ =
    nh_->advertiseService("enable_balance_constraint",
                          &BodyBalancer::enableBalanceConstraint, this);
  srv_go_to_balance_ =
    nh_->advertiseService("goto_balance",
                          &BodyBalancer::goToBalance, this);
  srv_enable_effector_control_ =
    nh_->advertiseService("enable_effector_control",
                          &BodyBalancer::enableEffectorControl, this);
  srv_set_effector_control_ =
    nh_->advertiseService("set_effector_control",
                          &BodyBalancer::setEffectorControl, this);
  srv_enable_effector_optimization_ =
    nh_->advertiseService("enable_effector_optimization",
                          &BodyBalancer::enableEffectorOptimization, this);
  isBalancing_.data = false;
  balance_pub_.publish(isBalancing_);
}

bool BodyBalancer::enableBalance(motion_msgs::Enable::Request& req,
                                 motion_msgs::Enable::Response& res) {
  mProxy_.wbEnable(req.is_enabled);
  isBalancing_.data = req.is_enabled;
  balance_pub_.publish(isBalancing_);
  return true;
}

bool BodyBalancer::footState(motion_msgs::FootState::Request& req,
                             motion_msgs::FootState::Response& res) {
  mProxy_.wbFootState(req.state_name, req.support_leg);
  return true;
}

bool BodyBalancer::enableBalanceConstraint(
  motion_msgs::EnableBalanceConstraint::Request& req,
  motion_msgs::EnableBalanceConstraint::Response& res) {
  mProxy_.wbEnableBalanceConstraint(req.is_enabled, req.support_leg);
  return true;
}

bool BodyBalancer::goToBalance(motion_msgs::GoToBalance::Request& req,
                               motion_msgs::GoToBalance::Response& res) {
  mProxy_.wbGoToBalance(req.support_leg, req.duration);
  return true;
}

bool BodyBalancer::enableEffectorControl(
  motion_msgs::EnableEffector::Request& req,
  motion_msgs::EnableEffector::Response& res) {
  mProxy_.wbEnableEffectorControl(req.effector_name, req.is_enabled);
  return true;
}

bool BodyBalancer::setEffectorControl(
  motion_msgs::SetEffectorControl::Request& req,
  motion_msgs::SetEffectorControl::Response& res) {
  mProxy_.wbSetEffectorControl(req.effector_name, req.target_coordinate);
  return true;
}

bool BodyBalancer::enableEffectorOptimization(
  motion_msgs::EnableEffector::Request& req,
  motion_msgs::EnableEffector::Response& res) {
  mProxy_.wbEnableEffectorOptimization(req.effector_name, req.is_enabled);
  return true;
}

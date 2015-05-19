/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-10
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI Whole Body Balancer
*/

#include "body_balancer.h"

BodyBalancer::BodyBalancer(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy) {
  nh_ = nh;
  mProxy_ = mProxy;
  isBalancing_ = false;
  INFO("Setting up Body Balancer publishers" << std::endl);
  balance_pub_ = nh_->advertise<std_msgs::Bool>("is_balancing", 10);

  INFO("Setting up Body Balancer services" << std::endl);
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
}

BodyBalancer::~BodyBalancer() {
  ros::shutdown();
}

void BodyBalancer::spinTopics() {
  std_msgs::Bool msg;
  msg.data = isBalancing_;
  balance_pub_.publish(msg);
}

bool BodyBalancer::enableBalance(motion::Enable::Request &req,
                                 motion::Enable::Response &res) {
  mProxy_->wbEnable(req.is_enabled);
  isBalancing_ = req.is_enabled;
  return true;
}

bool BodyBalancer::footState(motion::FootState::Request &req,
                             motion::FootState::Response &res) {
  mProxy_->wbFootState(req.state_name, req.support_leg);
  return true;
}

bool BodyBalancer::enableBalanceConstraint(
  motion::EnableBalanceConstraint::Request &req,
  motion::EnableBalanceConstraint::Response &res) {
  mProxy_->wbEnableBalanceConstraint(req.is_enabled, req.support_leg);
  return true;
}

bool BodyBalancer::goToBalance(motion::GoToBalance::Request &req,
                               motion::GoToBalance::Response &res) {
  mProxy_->wbGoToBalance(req.support_leg, req.duration);
  return true;
}

bool BodyBalancer::enableEffectorControl(
  motion::EnableEffector::Request &req,
  motion::EnableEffector::Response &res) {
  mProxy_->wbEnableEffectorControl(req.effector_name, req.is_enabled);
  return true;
}

bool BodyBalancer::setEffectorControl(
  motion::SetEffectorControl::Request &req,
  motion::SetEffectorControl::Response &res) {
  mProxy_->wbSetEffectorControl(req.effector_name, req.target_coordinate);
  return true;
}

bool BodyBalancer::enableEffectorOptimization(
  motion::EnableEffector::Request &req,
  motion::EnableEffector::Response &res) {
  mProxy_->wbEnableEffectorOptimization(req.effector_name, req.is_enabled);
  return true;
}

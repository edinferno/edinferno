/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-10
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI Whole Body Balancer
*/

#include "body_balancer.h"

Body_Balancer::Body_Balancer(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy) {

  nh_ = nh;
  mProxy_ = mProxy;
  isBalancing_ = false;
  INFO("Setting up Body Balancer publishers" << std::endl);
  balance_pub_ = nh_->advertise<std_msgs::Bool>("isBalancing", 10);

  INFO("Setting up Body Balancer services" << std::endl);
  srv_enable_balance_ = nh_->advertiseService(
                          "enableBalance",
                          &Body_Balancer::enableBalance, this);
  srv_foot_state_ = nh_->advertiseService(
                      "footState",
                      &Body_Balancer::footState, this);
  srv_enable_balance_constraint_ = nh_->advertiseService(
                                     "enableBalanceConstraint",
                                     &Body_Balancer::enableBalanceConstraint, this);
  srv_go_to_balance_ = nh_->advertiseService(
                         "goToBalance",
                         &Body_Balancer::goToBalance, this);
  srv_enable_effector_control_ = nh_->advertiseService(
                                   "enableEffectorControl",
                                   &Body_Balancer::enableEffectorControl, this);
  srv_set_effector_control_ = nh_->advertiseService(
                                "setEffectorControl",
                                &Body_Balancer::setEffectorControl, this);
  srv_enable_effector_optimization_ = nh_->advertiseService(
                                        "EnableEffectorOptimization",
                                        &Body_Balancer::enableEffectorOptimization, this);
}

Body_Balancer::~Body_Balancer() {
  ros::shutdown();
}

void Body_Balancer::spinTopics() {
  std_msgs::Bool msg;
  msg.data = isBalancing_;
  balance_pub_.publish(msg);
}

bool Body_Balancer::enableBalance(motion::enable::Request &req,
                                  motion::enable::Response &res) {
  mProxy_->wbEnable(req.isEnabled);
  isBalancing_ = req.isEnabled;
  return true;
}

bool Body_Balancer::footState(motion::footState::Request &req,
                              motion::footState::Response &res) {
  mProxy_->wbFootState(req.stateName, req.supportLeg);
  return true;
}

bool Body_Balancer::enableBalanceConstraint(
  motion::enableBalanceConstraint::Request &req,
  motion::enableBalanceConstraint::Response &res) {
  mProxy_->wbEnableBalanceConstraint(req.isEnabled, req.supportLeg);
  return true;
}

bool Body_Balancer::goToBalance(motion::goToBalance::Request &req,
                                motion::goToBalance::Response &res) {
  mProxy_->wbGoToBalance(req.supportLeg, req.duration);
  return true;
}

bool Body_Balancer::enableEffectorControl(
  motion::enableEffector::Request &req,
  motion::enableEffector::Response &res) {
  mProxy_->wbEnableEffectorControl(req.effectorName, req.isEnabled);
  return true;
}

bool Body_Balancer::setEffectorControl(motion::setEffectorControl::Request &req,
                                       motion::setEffectorControl::Response &res) {
  mProxy_->wbSetEffectorControl(req.effectorName, req.targetCoordinate);
  return true;
}

bool Body_Balancer::enableEffectorOptimization(motion::enableEffector::Request
    &req,
    motion::enableEffector::Response &res) {
  mProxy_->wbEnableEffectorOptimization(req.effectorName, req.isEnabled);
  return true;
}
/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-10
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI ALRobotPosture
*/

#include "robot_posture.h"

RobotPosture::RobotPosture(ros::NodeHandle* nh,
                           AL::ALRobotPostureProxy* pProxy) {
  nh_ = nh;
  pProxy_ = pProxy;
  INFO("Setting up Robot Posture services" << std::endl);
  srv_get_posture_list_ = nh_->advertiseService("get_posture_list",
                          &RobotPosture::getPostureList, this);
  srv_go_to_posture_ = nh_->advertiseService("goto_posture",
                       &RobotPosture::goToPosture, this);
  srv_apply_posture_ = nh_->advertiseService("apply_posture",
                       &RobotPosture::applyPosture, this);
  srv_stop_posture_ = nh_->advertiseService("stop_posture",
                      &RobotPosture::stopPosture, this);
  srv_get_posture_family_ = nh_->advertiseService("get_posture_family",
                            &RobotPosture::getPostureFamily, this);
  srv_get_posture_family_list_ = nh_->advertiseService("get_posture_family_list",
                                 &RobotPosture::getPostureFamilyList, this);
  srv_set_max_try_number_ = nh_->advertiseService("set_max_try_number",
                            &RobotPosture::setMaxTryNumber, this);
}

RobotPosture::~RobotPosture() {
  ros::shutdown();
}


bool RobotPosture::getPostureList(motion::GetPostureList::Request &req,
                                  motion::GetPostureList::Response &res) {
  res.posture_list = pProxy_->getPostureList();
  return true;
}

bool RobotPosture::goToPosture(motion::SetPosture::Request &req,
                               motion::SetPosture::Response &res) {
  res.success = pProxy_->goToPosture(req.posture_name, req.speed);
  return true;
}

bool RobotPosture::applyPosture(motion::SetPosture::Request &req,
                                motion::SetPosture::Response &res) {
  res.success = pProxy_->applyPosture(req.posture_name, req.speed);
  return true;
}

bool RobotPosture::stopPosture(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res) {
  pProxy_->stopMove();
  return true;
}

bool RobotPosture::getPostureFamily(motion::GetPostureFamily::Request &req,
                                    motion::GetPostureFamily::Response &res) {
  res.posture_family = pProxy_->getPostureFamily();
  return true;
}

bool RobotPosture::getPostureFamilyList(motion::GetPostureList::Request &req,
                                        motion::GetPostureList::Response &res) {
  res.posture_list = pProxy_->getPostureFamilyList();
  return true;
}

bool RobotPosture::setMaxTryNumber(motion::SetMaxTryNumber::Request &req,
                                   motion::SetMaxTryNumber::Response &res) {
  pProxy_->setMaxTryNumber(req.max_try_number);
  return true;
}


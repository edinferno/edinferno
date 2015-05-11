/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-10
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI ALRobotPosture
*/
#include "robot_posture.h"

Robot_Posture::Robot_Posture(ros::NodeHandle* nh,
                             AL::ALRobotPostureProxy* pProxy) {
  nh_ = nh;
  pProxy_ = pProxy;
  INFO("Setting up Robot Posture services" << std::endl);
  srv_get_posture_list_ = nh_->advertiseService("getPostureList",
                          &Robot_Posture::getPostureList, this);
  srv_go_to_posture_ = nh_->advertiseService("goToPosture",
                       &Robot_Posture::goToPosture, this);
  srv_apply_posture_ = nh_->advertiseService("applyPosture",
                       &Robot_Posture::applyPosture, this);
  srv_stop_posture_ = nh_->advertiseService("stopPosture",
                      &Robot_Posture::stopPosture, this);
  srv_get_posture_family_ = nh_->advertiseService("getPostureFamily",
                            &Robot_Posture::getPostureFamily, this);
  srv_get_posture_family_list_ = nh_->advertiseService("getPostureFamilyList",
                                 &Robot_Posture::getPostureFamilyList, this);
  srv_set_max_try_number_ = nh_->advertiseService("setMaxTryNumber",
                            &Robot_Posture::setMaxTryNumber, this);

}

Robot_Posture::~Robot_Posture() {
  ros::shutdown();
}


bool Robot_Posture::getPostureList(motion::getPostureList::Request &req,
                                   motion::getPostureList::Response &res) {
  res.postureList = pProxy_->getPostureList();
  return true;
}

bool Robot_Posture::goToPosture(motion::setPosture::Request &req,
                                motion::setPosture::Response &res) {
  res.success = pProxy_->goToPosture(req.postureName, req.speed);
  return true;
}

bool Robot_Posture::applyPosture(motion::setPosture::Request &req,
                                 motion::setPosture::Response &res) {
  res.success = pProxy_->applyPosture(req.postureName, req.speed);
  return true;
}

bool Robot_Posture::stopPosture(std_srvs::Empty::Request &req,
                                std_srvs::Empty::Response &res) {
  pProxy_->stopMove();
  return true;
}

bool Robot_Posture::getPostureFamily(motion::getPostureFamily::Request &req,
                                     motion::getPostureFamily::Response &res) {
  res.postureFamily = pProxy_->getPostureFamily();
  return true;
}

bool Robot_Posture::getPostureFamilyList(motion::getPostureList::Request &req,
    motion::getPostureList::Response &res) {
  res.postureList = pProxy_->getPostureFamilyList();
  return true;
}

bool Robot_Posture::setMaxTryNumber(motion::setMaxTryNumber::Request &req,
                                    motion::setMaxTryNumber::Response &res) {
  pProxy_->setMaxTryNumber(req.maxTryNumber);
  return true;
}


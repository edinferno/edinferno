/**
 * @file      robot_posture.cpp
 * @brief     ROS wrapper for NaoQI ALRobotPosture
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-22
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion/robot_posture.hpp"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <althread/alcriticalsection.h>

#include <qi/log.hpp>


RobotPosture::RobotPosture(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name): AL::ALModule(broker, name),
  fCallbackMutex(AL::ALMutex::createALMutex()) {
  qi::log::setVerbosity(qi::log::info);
  // setModuleDescription("Robot posture module.");
}

RobotPosture::~RobotPosture() {
}

void RobotPosture::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());
    pProxy_ = AL::ALRobotPostureProxy(getParentBroker());
  } catch (const AL::ALError& e) {
    ROS_ERROR_STREAM(e.what());
  }
}

void RobotPosture::rosSetup(ros::NodeHandle* nh) {
  nh_ = nh;
  ROS_INFO_STREAM("Setting up Robot Posture services");
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

bool RobotPosture::getPostureList(motion_msgs::GetPostureList::Request& req,
                                  motion_msgs::GetPostureList::Response& res) {
  res.posture_list = pProxy_.getPostureList();
  return true;
}

bool RobotPosture::goToPosture(motion_msgs::SetPosture::Request& req,
                               motion_msgs::SetPosture::Response& res) {
  res.success = pProxy_.goToPosture(req.posture_name, req.speed);
  return true;
}

bool RobotPosture::applyPosture(motion_msgs::SetPosture::Request& req,
                                motion_msgs::SetPosture::Response& res) {
  res.success = pProxy_.applyPosture(req.posture_name, req.speed);
  return true;
}

bool RobotPosture::stopPosture(std_srvs::Empty::Request& req,
                               std_srvs::Empty::Response& res) {
  pProxy_.stopMove();
  return true;
}

bool RobotPosture::getPostureFamily(
  motion_msgs::GetPostureFamily::Request& req,
  motion_msgs::GetPostureFamily::Response& res) {
  res.posture_family = pProxy_.getPostureFamily();
  return true;
}

bool RobotPosture::getPostureFamilyList(
  motion_msgs::GetPostureList::Request&  req,
  motion_msgs::GetPostureList::Response& res) {
  res.posture_list = pProxy_.getPostureFamilyList();
  return true;
}

bool RobotPosture::setMaxTryNumber(
  motion_msgs::SetMaxTryNumber::Request& req,
  motion_msgs::SetMaxTryNumber::Response& res) {
  pProxy_.setMaxTryNumber(req.max_try_number);
  return true;
}


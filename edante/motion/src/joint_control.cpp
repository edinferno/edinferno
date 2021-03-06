/**
 * @file      joint_control.cpp
 * @brief     Defines the Joint Control Wrapper functions
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-22
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion/joint_control.hpp"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <althread/alcriticalsection.h>

#include <qi/log.hpp>

JointControl::JointControl(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name): AL::ALModule(broker, name),
  fCallbackMutex(AL::ALMutex::createALMutex()) {
  qi::log::setVerbosity(qi::log::info);
  // setModuleDescription("Joint control module.");
}

JointControl::~JointControl() {
}

void JointControl::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());
    mProxy_ = AL::ALMotionProxy(getParentBroker());
  } catch (const AL::ALError& e) {
    ROS_ERROR_STREAM(e.what());
  }
}

void JointControl::rosSetup(ros::NodeHandle* nh) {
  nh_ = nh;
  ROS_INFO_STREAM("Setting up Joint Control services");

  srv_angle_interp_ = nh_->advertiseService("angle_interp",
                                            &JointControl::angleInterp, this);
  srv_angle_interp_speed_ = nh_->advertiseService("angle_interp_speed",
                                                  &JointControl::angleInterpSpeed, this);
  srv_set_angles_ = nh_->advertiseService("set_angles",
                                          &JointControl::setAngles, this);
  srv_change_angles_ = nh_->advertiseService("change_angles",
                                             &JointControl::changeAngles, this);
  srv_get_angles_ = nh_->advertiseService("get_angles",
                                          &JointControl::getAngles, this);
  srv_close_hand_ = nh_->advertiseService("close_hand",
                                          &JointControl::closeHand, this);
  srv_open_hand_ = nh_->advertiseService("open_hand",
                                         &JointControl::openHand, this);
}

bool JointControl::angleInterp(motion_msgs::AngleInterp::Request& req,
                               motion_msgs::AngleInterp::Response& res) {
  size_t s = req.names.size();

  AL::ALValue angle_lists;
  angle_lists.arraySetSize(s);
  AL::ALValue time_lists;
  time_lists.arraySetSize(s);

  for (size_t i = 0; i < s; ++i) {
    angle_lists[i] = req.angle_lists[i].float_list;
    time_lists[i] = req.time_lists[i].float_list;
  }

  try {
    mProxy_.post.angleInterpolation(req.names, angle_lists,
                                    time_lists, req.is_absolute);
    res.res = true;
  } catch (const std::exception& e) {
    res.res = false;
  }
  return true;
}
bool JointControl::angleInterpSpeed(
  motion_msgs::AngleInterpSpeed::Request& req,
  motion_msgs::AngleInterpSpeed::Response& res) {
  try {
    mProxy_.post.angleInterpolationWithSpeed(req.names, req.target_angles,
                                             req.max_speed_fraction);
    res.res = true;
  } catch (const std::exception& e) {
    res.res = false;
  }
  return true;
}

bool JointControl::setAngles(motion_msgs::SetAngles::Request& req,
                             motion_msgs::SetAngles::Response& res) {
  try {
    mProxy_.post.setAngles(req.names, req.angles, req.fraction_max_speed);
    res.res = true;
  } catch (const std::exception& e) {
    res.res = false;
  }
  return true;
}

bool JointControl::changeAngles(motion_msgs::ChangeAngles::Request& req,
                                motion_msgs::ChangeAngles::Response& res) {
  try {
    mProxy_.post.changeAngles(req.names, req.changes, req.fraction_max_speed);
    res.res = true;
  } catch (const std::exception& e) {
    res.res = false;
  }
  return true;
}

bool JointControl::getAngles(motion_msgs::GetAngles::Request& req,
                             motion_msgs::GetAngles::Response& res) {
  try {
    res.joint_angles = mProxy_.getAngles(req.names, req.use_sensors);
  } catch (const std::exception& e) {
    return false;
  }
  return true;
}

bool JointControl::closeHand(motion_msgs::UseHand::Request& req,
                             motion_msgs::UseHand::Response& res) {
  try {
    mProxy_.post.closeHand(req.hand_name);
    res.res = true;
  } catch (const std::exception& e) {
    res.res = false;
  }
  return true;
}
bool JointControl::openHand(motion_msgs::UseHand::Request& req,
                            motion_msgs::UseHand::Response& res) {
  try {
    mProxy_.post.openHand(req.hand_name);
    res.res = true;
  } catch (const std::exception& e) {
    res.res = false;
  }
  return true;
}

/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-18
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Defines the Joint Control Wrapper functions
*/

#include "joint_control.h"

JointControl::JointControl(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy) {
  nh_ = nh;
  mProxy_ = mProxy;

  INFO("Setting up Joint Control services" << std::endl);

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

JointControl::~JointControl() {
  ros::shutdown();
}

bool JointControl::angleInterp(motion::AngleInterp::Request &req,
                               motion::AngleInterp::Response &res) {
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
    mProxy_->post.angleInterpolation(req.names, angle_lists,
                                     time_lists, req.is_absolute);
    res.res = true;
  } catch (const std::exception& e) {
    res.res = false;
  }
  return true;
}
bool JointControl::angleInterpSpeed(motion::AngleInterpSpeed::Request &req,
                                    motion::AngleInterpSpeed::Response &res) {
  try {
    mProxy_->post.angleInterpolationWithSpeed(req.names, req.target_angles,
        req.max_speed_fraction);
    res.res = true;
  } catch (const std::exception& e) {
    res.res = false;
  }
  return true;
}

bool JointControl::setAngles(motion::SetAngles::Request &req,
                             motion::SetAngles::Response &res) {
  try {
    mProxy_->setAngles(req.names, req.angles, req.fraction_max_speed);
    res.res = true;
  } catch (const std::exception& e) {
    res.res = false;
  }
  return true;
}

bool JointControl::changeAngles(motion::ChangeAngles::Request &req,
                                motion::ChangeAngles::Response &res) {
  try {
    mProxy_->changeAngles(req.names, req.changes, req.fraction_max_speed);
    res.res = true;
  } catch (const std::exception& e) {
    res.res = false;
  }
  return true;
}

bool JointControl::getAngles(motion::GetAngles::Request &req,
                             motion::GetAngles::Response &res) {
  try {
    res.joint_angles = mProxy_->getAngles(req.names, req.use_sensors);
  } catch (const std::exception& e) {
    return false;
  }
  return true;
}

bool JointControl::closeHand(motion::UseHand::Request &req,
                             motion::UseHand::Response &res) {
  try {
    mProxy_->closeHand(req.hand_name);
    res.res = true;
  } catch (const std::exception& e) {
    res.res = false;
  }
  return true;
}
bool JointControl::openHand(motion::UseHand::Request &req,
                            motion::UseHand::Response &res) {
  try {
    mProxy_->openHand(req.hand_name);
    res.res = true;
  } catch (const std::exception& e) {
    res.res = false;
  }
  return true;
}

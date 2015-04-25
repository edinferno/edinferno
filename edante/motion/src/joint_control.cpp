/* 
* @Author: Alejandro Bordallo
* @Date:   2015-04-22 18:12:31
* @Last Modified by:   Alejandro Bordallo
* @Last Modified time: 2015-04-22 17:29:47
*/

#include "joint_control.h"

Joint_Control::Joint_Control()
{
  nh_ = new ros::NodeHandle();
  mProxy_ = new AL::ALMotionProxy("127.0.0.1", 9559);

  // INFO("Setting up Joint Control publishers" << std::endl);

  // INFO("Setting up Joint Control subscribers" << std::endl);

  INFO("Setting up Joint Control services" << std::endl);

  srv_angle_interp_ = nh_->advertiseService(
                  "motion/angleInterp", &Joint_Control::angleInterp, this);
  srv_angle_interp_speed_ = nh_->advertiseService(
                  "motion/angleInterpSpeed", &Joint_Control::angleInterpSpeed, this);
  srv_set_angles_ = nh_->advertiseService(
                  "motion/setAngles", &Joint_Control::setAngles, this);
  srv_change_angles_ = nh_->advertiseService(
                  "motion/changeAngles", &Joint_Control::changeAngles, this);
  srv_get_angles_ = nh_->advertiseService(
                  "motion/getAngles", &Joint_Control::getAngles, this);
  srv_close_hand_ = nh_->advertiseService(
                  "motion/closeHand", &Joint_Control::closeHand, this);
  srv_open_hand_ = nh_->advertiseService(
                  "motion/openHand", &Joint_Control::openHand, this);
}

Joint_Control::~Joint_Control()
{
  ros::shutdown();
  delete nh_;
}

bool Joint_Control::angleInterp(motion::angleInterp::Request &req,
                                motion::angleInterp::Response &res)
{
  int s = req.names.size();
  AL::ALValue names = req.names;

  AL::ALValue angleLists;
  angleLists.arraySetSize(s);
  AL::ALValue timeLists;
  timeLists.arraySetSize(s);

  for(unsigned i = 0; i < s; ++i) {
    angleLists[i] = req.angleLists[i].floatList;
    timeLists[i] = req.timeLists[i].floatList;
  }

  try{
    mProxy_->angleInterpolation(names, angleLists,
                                timeLists, true);
    res.res = true;
  }
  catch (const std::exception& e){
    res.res = false;
  }
  return true;
}
bool Joint_Control::angleInterpSpeed(motion::angleInterpSpeed::Request &req,
                                     motion::angleInterpSpeed::Response &res)
{
  return true;
}

bool Joint_Control::setAngles(motion::setAngles::Request &req,
                              motion::setAngles::Response &res)
{
  try{
    mProxy_->setAngles(req.names, req.angles, req.fractionMaxSpeed);
    res.res = true;
  }
  catch (const std::exception& e){
    res.res = false;
  }
  return true;
}

bool Joint_Control::changeAngles(motion::changeAngles::Request &req,
                                 motion::changeAngles::Response &res)
{
  try{
    mProxy_->changeAngles(req.names, req.changes, req.fractionMaxSpeed);
    res.res = true;
  }
  catch (const std::exception& e){
    res.res = false;
  }
  return true;
}

bool Joint_Control::getAngles(motion::getAngles::Request &req,
                              motion::getAngles::Response &res)
{
  try{
    res.jointAngles = mProxy_->getAngles(req.names, req.useSensors);
  }
  catch (const std::exception& e){
    return false;
  }
  return true;
}

bool Joint_Control::closeHand(motion::useHand::Request &req,
                              motion::useHand::Response &res)
{
  try{
    mProxy_->closeHand(req.handName);
    res.res = true;
  }
  catch (const std::exception& e){
    res.res = false;
  }
  return true;
}
bool Joint_Control::openHand(motion::useHand::Request &req,
                             motion::useHand::Response &res)
{
  try{
    mProxy_->openHand(req.handName);
    res.res = true;
  }
  catch (const std::exception& e){
    res.res = false;
  }
  return true;
}
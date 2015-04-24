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
  srv_close_hand_ = nh_->advertiseService("motion/closeHand",
                                           &Joint_Control::closeHand, this);
  srv_open_hand_ = nh_->advertiseService("motion/openHand",
                                         &Joint_Control::openHand, this);
}

Joint_Control::~Joint_Control()
{
  ros::shutdown();
  delete nh_;
}

bool Joint_Control::closeHand(motion::handName::Request &req,
                              motion::handName::Response &res)
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
bool Joint_Control::openHand(motion::handName::Request &req,
                             motion::handName::Response &res)
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
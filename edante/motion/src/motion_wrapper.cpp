/* 
* @File: motion_wrapper.cpp
* @Author: Alejandro Bordallo
* @Date:   2015-04-04 20:47:59
* @Last Modified by:   Alejandro
* @Last Modified time: 2015-04-07 19:14:13
* @Desc: Declares the Motion Wrapper functions
*/

#include "motion_wrapper.h"

Motion::Motion(int argc, char *argv[])
{
  ros::init(argc, argv, "motion");
  INFO("Setting up Nao motion publishers" << std::endl);
  wake_pub_ = nh_.advertise<std_msgs::Bool>("/isAwake", 10);
  // INFO("Setting up Nao motion subscribers" << std::endl);
  INFO("Setting up Nao motion services" << std::endl);  
  // set_stiffness_ = nh_.advertiseService("setStiffnesses", 
  //                                       &Motion::recStiffness, this);

  mProxy_ = new AL::ALMotionProxy("127.0.0.1", 9559);
  awake_ = false;
}

Motion::~Motion()
{
  ros::shutdown();  
}

void Motion::wakeUp()
{
  mProxy_->wakeUp();
  awake_ = true;
}

void Motion::rest()
{
  mProxy_->rest();
  awake_ = false;
}

bool Motion::recStiffness(motion::setStiffness::Request &req, 
                          motion::setStiffness::Response &res)
{
  this->setStiffnesses(req.names, req.stiffnesses);
  return true;
}

void Motion::setStiffnesses(const vector<string>& names, 
                            const vector<float>& stiffnesses)
{
  mProxy_->setStiffnesses(names, stiffnesses);
}

vector<float> Motion::getStiffnesses(const vector<string>& names)
{
  return mProxy_->getStiffnesses(names);
}

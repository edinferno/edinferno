/* 
* @File: motion_wrapper.cpp
* @Author: Alejandro Bordallo
* @Date:   2015-04-04 20:47:59
* @Last Modified by:   Alejandro
* @Last Modified time: 2015-04-07 16:52:03
* @Desc: Declares the Motion Wrapper functions
*/

#include "motion_wrapper.h"

Motion::Motion(int argc, char *argv[])
{
  ros::init(argc, argv, "motion");
  wake_pub_ = nh_.advertise<std_msgs::Bool>("/isAwake", 10);
  INFO("Setting up Nao motion publishers" << std::endl);
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

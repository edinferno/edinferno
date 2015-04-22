#include "locomotion_control.h"

Locomotion_Control::Locomotion_Control()
{
  nh_ = new ros::NodeHandle();
  mProxy_ = new AL::ALMotionProxy("127.0.0.1", 9559);
  INFO("Setting up Nao locomotion publishers" << std::endl);
  moving_pub_ = nh_->advertise<std_msgs::Bool>("motion/isMoving", 10); 
  moving_ = false; 
}

Locomotion_Control::~Locomotion_Control()
{
  ros::shutdown();  
}

void Locomotion_Control::moveInit()
{
	mProxy_->moveInit();
	moving_ = true;
}

void Locomotion_Control::spinTopics()
{
  std_msgs::Bool msg;
  msg.data = moving_;
  moving_pub_.publish(msg);
}
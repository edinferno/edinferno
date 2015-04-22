#include "locomotion_control.h"

Locomotion_Control::Locomotion_Control()
{
  nh_ = new ros::NodeHandle();
  mProxy_ = new AL::ALMotionProxy("127.0.0.1", 9559);
  INFO("Setting up Nao locomotion publishers" << std::endl);
  moving_pub_ = nh_->advertise<std_msgs::Bool>("motion/isMoving", 10); 
  INFO("Setting up Nao motion publishers" << std::endl);
  srv_moveInit_ = nh_->advertiseService("motion/moveInit", &Locomotion_Control::moveInit, this);
  moving_ = false; 
}

Locomotion_Control::~Locomotion_Control()
{
  ros::shutdown();  
}

bool Locomotion_Control::moveInit(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res)
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
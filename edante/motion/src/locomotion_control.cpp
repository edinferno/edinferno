#include "locomotion_control.h"

Locomotion_Control::Locomotion_Control(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy)
{
  nh_ = nh;
  mProxy_ = mProxy;
  INFO("Setting up Nao locomotion publishers" << std::endl);
  moving_pub_ = nh_->advertise<std_msgs::Bool>("motion/isMoving", 10); 
  INFO("Setting up Nao motion publishers" << std::endl);
  srv_moveInit_ = nh_->advertiseService("motion/moveInit", &Locomotion_Control::moveInit, this);
  srv_waitMoveFinished_ = nh_->advertiseService("motion/waitMoveFinish", &Locomotion_Control::waitUntilMoveIsFinished, this);
  srv_stopMove_ = nh_->advertiseService("motion/stopMove", &Locomotion_Control::stopMove, this);
  srv_getRobotPosition = nh_->advertiseService("motion/getRobotPosition", &Locomotion_Control::getRobotPosition, this);
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
	moving_ = moveIsActive();
  return true;
}

 bool Locomotion_Control::waitUntilMoveIsFinished(
                                    std_srvs::Empty::Request &req, 
                                    std_srvs::Empty::Response &res)
 {
  mProxy_->waitUntilMoveIsFinished();
  return true;
 }

bool Locomotion_Control::moveIsActive()
{
  mProxy_->moveIsActive();
  return true;
}

bool Locomotion_Control::stopMove(std_srvs::Empty::Request &req, 
                                  std_srvs::Empty::Response &res)
{
  mProxy_->stopMove();
  moving_ = moveIsActive();
  return true;
}

bool Locomotion_Control::getRobotPosition(motion::getRobotPosition::Request &req,
                                          motion::getRobotPosition::Response &res)
{
  bool useSensors = req.useSensors;
  res.positions = mProxy_->getRobotPosition(useSensors);
  return true;
}

void Locomotion_Control::spinTopics()
{
  std_msgs::Bool msg;
  msg.data = moving_;
  moving_pub_.publish(msg);
}

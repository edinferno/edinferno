#include "locomotion_control.h"

Locomotion_Control::Locomotion_Control(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy)
{
  nh_ = nh;
  mProxy_ = mProxy;
  INFO("Setting up Nao locomotion publishers" << std::endl);
  moving_pub_ = nh_->advertise<std_msgs::Bool>("isMoving", 10); 
  INFO("Setting up Nao motion publishers" << std::endl);
  srv_moveInit_ = nh_->advertiseService("moveInit", &Locomotion_Control::moveInit, this);
  srv_waitMoveFinished_ = nh_->advertiseService("waitMoveFinish", &Locomotion_Control::waitUntilMoveIsFinished, this);
  srv_stopMove_ = nh_->advertiseService("stopMove", &Locomotion_Control::stopMove, this);
  srv_getRobotPosition = nh_->advertiseService("getRobotPosition", &Locomotion_Control::getRobotPosition, this);
  srv_getNextRobotPosition = nh_->advertiseService("getNextRobotPosition", &Locomotion_Control::getNextRobotPosition, this);
  srv_getRobotVelocity = nh_->advertiseService("getRobotVelocity", &Locomotion_Control::getRobotVelocity, this);
  srv_getWalkArmsEnabled = nh_->advertiseService("getWalkArmsEnabled", &Locomotion_Control::getWalkArmsEnabled, this);
  srv_setWalkArmsEnabled = nh_->advertiseService("setWalkArmsEnabled", &Locomotion_Control::setWalkArmsEnabled, this);
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

bool Locomotion_Control::getNextRobotPosition(motion::getNextRobotPosition::Request &req,
                                              motion::getNextRobotPosition::Response &res)
{
  res.positions = mProxy_->getNextRobotPosition();
  return true;
}

bool Locomotion_Control::getRobotVelocity(motion::getRobotVelocity::Request &req,
                                          motion::getRobotVelocity::Response &res)
{
  res.velocity = mProxy_->getRobotVelocity();
  return true;
}

bool Locomotion_Control::getWalkArmsEnabled(motion::getWalkArmsEnabled::Request &req,
                                  motion::getWalkArmsEnabled::Response &res)
{
  AL::ALValue result = mProxy_->getWalkArmsEnabled();
  res.armMotions.resize(2);
  res.armMotions[0] = bool(result[0]);
  res.armMotions[1] = bool(result[1]);
  return true;
}

bool Locomotion_Control::setWalkArmsEnabled(motion::setWalkArmsEnabled::Request &req,
  motion::setWalkArmsEnabled::Response &res)
{
  bool left = req.leftArmEnable;
  bool right = req.rightArmEnable;
  mProxy_->setWalkArmsEnabled(left, right);
  return true;
}  

void Locomotion_Control::spinTopics()
{
  std_msgs::Bool msg;
  msg.data = moving_;
  moving_pub_.publish(msg);
}

#include "cartesian_control.h"

Cartesian_Control::Cartesian_Control(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy)
{
  nh_ = nh;
  mProxy_ = mProxy;
  INFO("Setting up Nao motion services" << std::endl);
  srv_get_position_ = nh_->advertiseService("getPosition",
  										&Cartesian_Control::getPosition, this);
  srv_get_transform_ = nh_->advertiseService("getTransform",
  										&Cartesian_Control::getTransform, this);
}

Cartesian_Control::~Cartesian_Control()
{
  ros::shutdown();
}

bool Cartesian_Control::getPosition(motion::getPosition::Request &req,
                          motion::getPosition::Response &res)
{
  DEBUG("Service: getPosition" << std::endl);

  string nameJoint = req.name;
  int space = req.space;
  bool useSensorValues = req.useSensorValues;
  res.position = mProxy_->getPosition(nameJoint, space, useSensorValues);
  return true;
}

bool Cartesian_Control::getTransform(motion::getTransform::Request &req,
                          motion::getTransform::Response &res)
{
  DEBUG("Service: getTransform" << std::endl);

  string nameJoint = req.name;
  int space = req.space;
  bool useSensorValues = req.useSensorValues;
  res.transform = mProxy_->getTransform(nameJoint, space, useSensorValues);
  return true;
}
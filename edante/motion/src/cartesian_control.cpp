#include "cartesian_control.h"

Cartesian_Control::Cartesian_Control()
{
  nh_ = new ros::NodeHandle();
  mProxy_ = new AL::ALMotionProxy("127.0.0.1", 9559);

  INFO("Setting up Nao motion services" << std::endl);
  get_position_ = nh_->advertiseService("motion/getPosition",
  										&Cartesian_Control::getPositionSrv, this);
  get_transform_ = nh_->advertiseService("motion/getTransform",
  										&Cartesian_Control::getTransformSrv, this);
}

Cartesian_Control::~Cartesian_Control()
{
  ros::shutdown();
  delete nh_;
}

bool Cartesian_Control::getPositionSrv(motion::getPosition::Request &req,
                          motion::getPosition::Response &res)
{
  DEBUG("Service: getPosition" << std::endl);

  string nameJoint = req.name;
  int space = req.space;
  bool useSensorValues = req.useSensorValues;
  res.position = this->getPosition(nameJoint, space, useSensorValues);
  return true;
}

vector<float> Cartesian_Control::getPosition(string& name, int& space, bool& useSensorValues)
{
  return mProxy_->getPosition(name, space, useSensorValues);
}

bool Cartesian_Control::getTransformSrv(motion::getTransform::Request &req,
                          motion::getTransform::Response &res)
{
  DEBUG("Service: getTransform" << std::endl);

  string nameJoint = req.name;
  int space = req.space;
  bool useSensorValues = req.useSensorValues;
  res.transform = this->getTransform(nameJoint, space, useSensorValues);
  return true;
}

vector<float> Cartesian_Control::getTransform(string& name, int& space, bool& useSensorValues)
{
  return mProxy_->getTransform(name, space, useSensorValues);
}

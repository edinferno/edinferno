/*
* @File: motion_wrapper.cpp
* @Author: Alejandro Bordallo
* @Date:   2015-04-04 20:47:59
* @Last Modified by:   Alejandro Bordallo
* @Last Modified time: 2015-04-15 17:22:34
* @Desc: Declares the Motion Wrapper functions
*/

#include "motion_wrapper.h"

Motion::Motion()
{
  nh_ = new ros::NodeHandle();
  INFO("Setting up Nao motion publishers" << std::endl);
  wake_pub_ = nh_->advertise<std_msgs::Bool>("/isAwake", 10);
  // INFO("Setting up Nao motion subscribers" << std::endl);
  INFO("Setting up Nao motion services" << std::endl);
  set_stiffness_ = nh_->advertiseService("setStiffness",
                                        &Motion::recStiffness, this);

  mProxy_ = new AL::ALMotionProxy("127.0.0.1", 9559);
  awake_ = false;
}

Motion::~Motion()
{
  ros::shutdown();
  delete nh_;
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

void Motion::spinTopics()
{
  std_msgs::Bool msg;
  msg.data = awake_;
  wake_pub_.publish(msg);
}

bool Motion::recStiffness(motion::setStiffness::Request &req,
                          motion::setStiffness::Response &res)
{
  // this->setStiffnesses(req.names, req.stiffnesses);
  DEBUG("Stiffness Received!");
  res.res = true;
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

void Motion::testSrv()
{
  ros::ServiceClient testClient = nh_->serviceClient<motion::setStiffness>("set_stiffness");
  motion::setStiffness srv;
  std::vector<string> test_names;
  test_names.push_back("RArm");
  srv.request.names = test_names;

  std::vector<float> test_stiffness;
  test_stiffness.push_back(1.0f);
  srv.request.stiffnesses = test_stiffness;

  if (testClient.call(srv))
  {
    INFO("Stiffness set");
  }
  else
  {
    ERR("Failed to call stiffness set!");
  }
}

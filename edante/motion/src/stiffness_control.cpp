/*
* @File: motion.cpp
* @Author: Alejandro Bordallo
* @Date:   2015-04-04 20:47:59
* @Last Modified by:   Alejandro Bordallo
* @Last Modified time: 2015-04-22 16:15:00
* @Desc: Declares the Stiffness Control Wrapper functions
*/

#include "stiffness_control.h"

Stiffness_Control::Stiffness_Control()
{
  nh_ = new ros::NodeHandle();
  mProxy_ = new AL::ALMotionProxy("127.0.0.1", 9559);

  INFO("Setting up Nao motion publishers" << std::endl);
  wake_pub_ = nh_->advertise<std_msgs::Bool>("motion/isAwake", 10);
  // INFO("Setting up Nao motion subscribers" << std::endl);

  INFO("Setting up Nao motion services" << std::endl);
  set_stiffness_ = nh_->advertiseService("motion/setStiffness",
                                        &Stiffness_Control::recStiffness, this);
  get_stiffness_ = nh_->advertiseService("motion/getStiffness",
                                        &Stiffness_Control::getStiffness, this);
  srv_wake_up_ = nh_->advertiseService("motion/wakeUp", &Stiffness_Control::wakeUp, this);
  srv_rest_ = nh_->advertiseService("motion/rest", &Stiffness_Control::rest, this);

  awake_ = false;
}

Stiffness_Control::~Stiffness_Control()
{
  ros::shutdown();
  delete nh_;
}

bool Stiffness_Control::wakeUp(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &res)
{
  mProxy_->wakeUp();
  awake_ = true;
}

bool Stiffness_Control::rest(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res)
{
  mProxy_->rest();
  awake_ = false;
}

void Stiffness_Control::spinTopics()
{
  std_msgs::Bool msg;
  msg.data = awake_;
  wake_pub_.publish(msg);
}

bool Stiffness_Control::recStiffness(motion::setStiffness::Request &req,
                          motion::setStiffness::Response &res)
{
  bool nameIsVect;
  string jointName;
  vector<string> jointNameVect;
  if (req.names.size() == 1){
    jointName = req.names.front();
    nameIsVect = false;
  }
  else{
    jointNameVect = req.names;
    nameIsVect = true;
  }

  bool stiffIsVect;
  float jointStiffness;
  vector<float> jointStiffnessVect; 
  if (req.stiffnesses.size() == 1){
    jointStiffness = req.stiffnesses.front();
    stiffIsVect = false;
  }
  else{
    jointStiffnessVect = req.stiffnesses;
    stiffIsVect = true;
  }

  bool succeeded = false;
  if (nameIsVect){
    if (stiffIsVect){
      succeeded = this->setStiffnesses(jointNameVect, jointStiffnessVect);
      INFO(succeeded);
    }
    else{
      succeeded = this->setStiffnesses(jointNameVect, jointStiffness);
      INFO(succeeded);
    }
  }
  else{
    if (stiffIsVect){
      succeeded = this->setStiffnesses(jointName, jointStiffnessVect);
      INFO(succeeded);
    }
    else{
      succeeded = this->setStiffnesses(jointName, jointStiffness);
      INFO(succeeded);
    }
  }
  res.res = succeeded;
  return true;
}

bool Stiffness_Control::setStiffnesses(string& name, float& stiffness)
{
  try{
    mProxy_->setStiffnesses(name, stiffness);
  }
  catch (const std::exception& e){
    return false;
  }
  return true;
}

bool Stiffness_Control::setStiffnesses(string& name, const vector<float>& stiffnesses)
{
  try{
    mProxy_->setStiffnesses(name, stiffnesses);
  }
  catch (const std::exception& e){
    return false;
  }
  return true;
}

bool Stiffness_Control::setStiffnesses(const vector<string>& names, float& stiffness)
{
  try{
    mProxy_->setStiffnesses(names, stiffness);
  }
  catch (const std::exception& e){
    return false;
  }
  return true;
}

bool Stiffness_Control::setStiffnesses(const vector<string>& names,
                            const vector<float>& stiffnesses)
{
  try{
    mProxy_->setStiffnesses(names, stiffnesses);
  }
  catch (const std::exception& e){
    return false;
  }
  return true;
}

bool Stiffness_Control::getStiffness(motion::getStiffness::Request &req,
                          motion::getStiffness::Response &res)
{
  vector<string> jointNameVect = req.names;
  res.stiffnesses = this->getStiffnesses(jointNameVect);
  return true;
}

vector<float> Stiffness_Control::getStiffnesses(const vector<string>& names)
{
  return mProxy_->getStiffnesses(names);
}

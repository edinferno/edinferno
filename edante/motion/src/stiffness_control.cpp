/*
* @File: motion.cpp
* @Author: Alejandro Bordallo
* @Date:   2015-04-04 20:47:59
* @Last Modified by:   Alejandro Bordallo
* @Last Modified time: 2015-04-22 16:15:00
* @Desc: Declares the Stiffness Control Wrapper functions
*/

#include "stiffness_control.h"

Stiffness_Control::Stiffness_Control(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy)
{
  nh_ = nh;
  mProxy_ = mProxy;
  INFO("Setting up Nao motion publishers" << std::endl);
  wake_pub_ = nh_->advertise<std_msgs::Bool>("isAwake", 10);

  INFO("Setting up Nao motion services" << std::endl);
  srv_wake_up_ = nh_->advertiseService("wakeUp",
                                       &Stiffness_Control::wakeUp, this);
  srv_rest_ = nh_->advertiseService("rest", 
                                    &Stiffness_Control::rest, this);
  stiffness_interp_=nh_->advertiseService("stiffnessInterpolation",
                                    &Stiffness_Control::stiffnessInterp, this);
  set_stiffness_ = nh_->advertiseService("setStiffness",
                                        &Stiffness_Control::secStiffness, this);
  get_stiffness_ = nh_->advertiseService("getStiffness",
                                        &Stiffness_Control::getStiffness, this);

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

 bool Stiffness_Control::stiffnessInterp(motion::stiffnessInterp::Request &req,
                                         motion::stiffnessInterp::Response &res)
{
  int s = req.names.size();
  AL::ALValue names = req.names;

  AL::ALValue stiffnessLists;
  stiffnessLists.arraySetSize(s);
  AL::ALValue timeLists;
  timeLists.arraySetSize(s);

  for(unsigned i = 0; i < s; ++i) {
    stiffnessLists[i] = req.stiffnessLists[i].floatList;
    timeLists[i] = req.timeLists[i].floatList;
  }

  try{
    mProxy_->stiffnessInterpolation(names, stiffnessLists, timeLists);
    res.res = true;
  }
  catch (const std::exception& e){
    res.res = false;
  }
  return true;
}

bool Stiffness_Control::secStiffness(motion::setStiffness::Request &req,
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
    }
    else{
      succeeded = this->setStiffnesses(jointNameVect, jointStiffness);
    }
  }
  else{
    if (stiffIsVect){
      succeeded = this->setStiffnesses(jointName, jointStiffnessVect);
    }
    else{
      succeeded = this->setStiffnesses(jointName, jointStiffness);
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

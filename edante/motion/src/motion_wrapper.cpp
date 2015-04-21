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
  mProxy_ = new AL::ALMotionProxy("127.0.0.1", 9559);

  INFO("Setting up Nao motion publishers" << std::endl);
  wake_pub_ = nh_->advertise<std_msgs::Bool>("motion/isAwake", 10);
  // INFO("Setting up Nao motion subscribers" << std::endl);

  INFO("Setting up Nao motion services" << std::endl);
  set_stiffness_ = nh_->advertiseService("motion/setStiffness",
                                        &Motion::recStiffness, this);
  get_stiffness_ = nh_->advertiseService("motion/getStiffness",
                                        &Motion::getStiffness, this);
  srv_awake_ = nh_->advertiseService("motion/awake", &Motion::wakeUp, this);
  srv_rest_ = nh_->advertiseService("motion/rest", &Motion::rest, this);

  awake_ = false;
}

Motion::~Motion()
{
  ros::shutdown();
  delete nh_;
}

bool Motion::wakeUp(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &res)
{
  mProxy_->wakeUp();
  awake_ = true;
}

bool Motion::rest(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res)
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
  DEBUG("Service: setStiffness" << std::endl);
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

bool Motion::setStiffnesses(string& name, float& stiffness)
{
  try{
    mProxy_->setStiffnesses(name, stiffness);
  }
  catch (const std::exception& e){
    return false;
  }
  return true;
}

bool Motion::setStiffnesses(string& name, const vector<float>& stiffnesses)
{
  try{
    mProxy_->setStiffnesses(name, stiffnesses);
  }
  catch (const std::exception& e){
    return false;
  }
  return true;
}

bool Motion::setStiffnesses(const vector<string>& names, float& stiffness)
{
  try{
    mProxy_->setStiffnesses(names, stiffness);
    }
  catch (const std::exception& e){
    return false;
  }
  return true;
}

bool Motion::setStiffnesses(const vector<string>& names,
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

bool Motion::getStiffness(motion::getStiffness::Request &req,
                          motion::getStiffness::Response &res)
{
  DEBUG("Service: getStiffness" << std::endl);

  vector<string> jointNameVect = req.names;
  res.stiffnesses = this->getStiffnesses(jointNameVect);
  return true;
}

vector<float> Motion::getStiffnesses(const vector<string>& names)
{
  return mProxy_->getStiffnesses(names);
}

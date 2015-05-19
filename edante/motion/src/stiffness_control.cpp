/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI Stiffness control API
*/

#include "stiffness_control.h"

StiffnessControl::StiffnessControl(ros::NodeHandle* nh,
                                   AL::ALMotionProxy* mProxy) {
  nh_ = nh;
  mProxy_ = mProxy;
  awake_ = false;
  INFO("Setting up Stiffness Control publishers" << std::endl);
  wake_pub_ = nh_->advertise<std_msgs::Bool>("is_awake", 10);

  INFO("Setting up Stiffness Control services" << std::endl);
  srv_wake_up_ = nh_->advertiseService("wake_up",
                                       &StiffnessControl::wakeUp, this);
  srv_rest_ = nh_->advertiseService("rest",
                                    &StiffnessControl::rest, this);
  stiffness_interp_ = nh_->advertiseService("stiffness_interpolation",
                      &StiffnessControl::stiffnessInterp, this);
  set_stiffness_ =
    nh_->advertiseService("set_stiffness",
                          &StiffnessControl::setStiffness, this);
  get_stiffness_ =
    nh_->advertiseService("get_stiffness",
                          &StiffnessControl::getStiffness, this);
}

StiffnessControl::~StiffnessControl() {
  ros::shutdown();
}

void StiffnessControl::spinTopics() {
  std_msgs::Bool msg;
  msg.data = awake_;
  wake_pub_.publish(msg);
}

bool StiffnessControl::wakeUp(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res) {
  mProxy_->wakeUp();
  awake_ = true;
  return true;
}

bool StiffnessControl::rest(std_srvs::Empty::Request &req,
                            std_srvs::Empty::Response &res) {
  mProxy_->rest();
  awake_ = false;
  return true;
}

bool StiffnessControl::stiffnessInterp(
  motion::StiffnessInterp::Request &req,
  motion::StiffnessInterp::Response &res) {
  size_t s = req.names.size();

  AL::ALValue stiffness_lists;
  stiffness_lists.arraySetSize(s);
  AL::ALValue time_lists;
  time_lists.arraySetSize(s);

  for (size_t i = 0; i < s; ++i) {
    stiffness_lists[i] = req.stiffness_lists[i].float_list;
    time_lists[i] = req.time_lists[i].float_list;
  }

  try {
    mProxy_->post.stiffnessInterpolation(req.names, stiffness_lists, time_lists);
    res.res = true;
  } catch (const std::exception& e) {
    res.res = false;
  }
  return true;
}

bool StiffnessControl::setStiffness(motion::SetStiffness::Request &req,
                                    motion::SetStiffness::Response &res) {
  bool nameIsVect;
  string jointName;
  vector<string> jointNameVect;
  if (req.names.size() == 1) {
    jointName = req.names.front();
    nameIsVect = false;
  } else {
    jointNameVect = req.names;
    nameIsVect = true;
  }

  bool stiffIsVect;
  float jointStiffness;
  vector<float> jointStiffnessVect;
  if (req.stiffnesses.size() == 1) {
    jointStiffness = req.stiffnesses.front();
    stiffIsVect = false;
  } else {
    jointStiffnessVect = req.stiffnesses;
    stiffIsVect = true;
  }

  bool succeeded = false;
  if (nameIsVect) {
    if (stiffIsVect) {
      succeeded = this->setStiffnesses(jointNameVect, jointStiffnessVect);
    } else {
      succeeded = this->setStiffnesses(jointNameVect, jointStiffness);
    }
  } else {
    if (stiffIsVect) {
      succeeded = this->setStiffnesses(jointName, jointStiffnessVect);
    } else {
      succeeded = this->setStiffnesses(jointName, jointStiffness);
    }
  }
  res.res = succeeded;
  return true;
}

bool StiffnessControl::getStiffness(motion::GetStiffness::Request &req,
                                    motion::GetStiffness::Response &res) {
  res.stiffnesses = mProxy_->getStiffnesses(req.names);
  return true;
}

bool StiffnessControl::setStiffnesses(const string& name,
                                      const float& stiffness) {
  try {
    mProxy_->setStiffnesses(name, stiffness);
  } catch (const std::exception& e) {
    return false;
  }
  return true;
}

bool StiffnessControl::setStiffnesses(const string& name,
                                      const vector<float>& stiffnesses) {
  try {
    mProxy_->setStiffnesses(name, stiffnesses);
  } catch (const std::exception& e) {
    return false;
  }
  return true;
}

bool StiffnessControl::setStiffnesses(const vector<string>& names,
                                      const float& stiffness) {
  try {
    mProxy_->setStiffnesses(names, stiffness);
  } catch (const std::exception& e) {
    return false;
  }
  return true;
}

bool StiffnessControl::setStiffnesses(const vector<string>& names,
                                      const vector<float>& stiffnesses) {
  try {
    mProxy_->setStiffnesses(names, stiffnesses);
  } catch (const std::exception& e) {
    return false;
  }
  return true;
}

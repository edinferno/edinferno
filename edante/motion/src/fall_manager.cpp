/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-11
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS Wrapper for NaoQI Fall Manager API
*/

#include "fall_manager.h"

Fall_Manager::Fall_Manager(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy) {
  nh_ = nh;
  mProxy_ = mProxy;
  INFO("Setting up Fall Manager services" << std::endl);

  srv_set_fall_manager_ = nh_->advertiseService("setFallManagerEnabled",
                          &Fall_Manager::setFallManagerEnabled, this);
  srv_get_fall_manager_ = nh_->advertiseService("getFallManagerEnabled",
                          &Fall_Manager::getFallManagerEnabled, this);
}

Fall_Manager::~Fall_Manager() {
  ros::shutdown();
}


// ROS services
bool Fall_Manager::setFallManagerEnabled(motion::enable::Request &req,
    motion::enable::Response &res) {
  mProxy_->setFallManagerEnabled(req.isEnabled);
  return true;
}

bool Fall_Manager::getFallManagerEnabled(motion::isEnabled::Request &req,
    motion::isEnabled::Response &res) {
  res.isEnabled = mProxy_->getFallManagerEnabled();
  return true;
}

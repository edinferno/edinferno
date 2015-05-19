/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-11
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS Wrapper for NaoQI Fall Manager API
*/

#include "fall_manager.h"

FallManager::FallManager(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy,
                         AL::ALMemoryProxy* memProxy) {
  nh_ = nh;
  mProxy_ = mProxy;
  memProxy_ = memProxy;
  INFO("Setting up Fall Manager publishers" << std::endl);
  has_fallen_pub_ = nh_->advertise<std_msgs::Bool>("hasFallen", 10);
  INFO("Setting up Fall Manager services" << std::endl);
  srv_set_fall_manager_ = nh_->advertiseService("set_fall_manager_enabled",
                          &FallManager::setFallManagerEnabled, this);
  srv_get_fall_manager_ = nh_->advertiseService("get_fall_manager_enabled",
                          &FallManager::getFallManagerEnabled, this);
}

FallManager::~FallManager() {
  ros::shutdown();
}

void FallManager::spinTopics() {
  // For some reason always true!
  std_msgs::Bool msg;
  if (static_cast<bool>(memProxy_->getData("robotHasFallen")) == true) {
    msg.data = true;
  } else {
    msg.data = false;
  }
  has_fallen_pub_.publish(msg);
}

// ROS services
bool FallManager::setFallManagerEnabled(motion::Enable::Request &req,
                                        motion::Enable::Response &res) {
  mProxy_->setFallManagerEnabled(req.is_enabled);
  return true;
}

bool FallManager::getFallManagerEnabled(motion::IsEnabled::Request &req,
                                        motion::IsEnabled::Response &res) {
  res.is_enabled = mProxy_->getFallManagerEnabled();
  return true;
}

/**
 * @file      fall_manager.cpp
 * @brief     ROS Wrapper for NaoQI Fall Manager API
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion/fall_manager.hpp"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <althread/alcriticalsection.h>

#include <qi/log.hpp>

FallManager::FallManager(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name): AL::ALModule(broker, name),
  fCallbackMutex(AL::ALMutex::createALMutex()) {
  qi::log::setVerbosity(qi::log::info);
  setModuleDescription("Fall Test module.");
  functionName("hasFallen", getName(), "Robot has fallen");
  BIND_METHOD(FallManager::hasFallen)
}

FallManager::~FallManager() {
  fMemoryProxy.unsubscribeToEvent("robotHasFallen", "FallManager");
}

void FallManager::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());
    mProxy_ = AL::ALMotionProxy(getParentBroker());
    fMemoryProxy.subscribeToEvent("robotHasFallen", "FallManager", "hasFallen");
  } catch (const AL::ALError& e) {
    ROS_ERROR_STREAM(e.what());
  }
}

void FallManager::rosSetup(ros::NodeHandle* nh) {
  nh_ = nh;
  ROS_INFO_STREAM("Setting up Fall Manager publishers");
  has_fallen_pub_ = nh_->advertise<std_msgs::Bool>("has_fallen", 10, true);
  ROS_INFO_STREAM("Setting up Fall Manager services");
  srv_set_fall_manager_ = nh_->advertiseService("set_fall_manager_enabled",
                                                &FallManager::setFallManagerEnabled, this);
  srv_get_fall_manager_ = nh_->advertiseService("get_fall_manager_enabled",
                                                &FallManager::getFallManagerEnabled, this);
  fallen_.data = false;
  has_fallen_pub_.publish(fallen_);
}

void FallManager::hasFallen() {
  AL::ALCriticalSection section(fCallbackMutex);
  fallen_.data = true;
  has_fallen_pub_.publish(fallen_);
}

// ROS services
bool FallManager::setFallManagerEnabled(motion_msgs::Enable::Request& req,
                                        motion_msgs::Enable::Response& res) {
  mProxy_.setFallManagerEnabled(req.is_enabled);
  return true;
}

bool FallManager::getFallManagerEnabled(motion_msgs::IsEnabled::Request& req,
                                        motion_msgs::IsEnabled::Response& res) {
  res.is_enabled = mProxy_.getFallManagerEnabled();
  return true;
}

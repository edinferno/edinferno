/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-11
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS Wrapper for NaoQI Fall Manager API
*/

#include "fall_manager.h"

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
    DEBUG(e.what() << std::endl);
  }
}

void FallManager::rosSetup(ros::NodeHandle* nh) {
  nh_ = nh;
  INFO("Setting up Fall Manager publishers" << std::endl);
  has_fallen_pub_ = nh_->advertise<std_msgs::Bool>("has_fallen", 10, true);
  INFO("Setting up Fall Manager services" << std::endl);
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
bool FallManager::setFallManagerEnabled(motion::Enable::Request &req,
                                        motion::Enable::Response &res) {
  mProxy_.setFallManagerEnabled(req.is_enabled);
  return true;
}

bool FallManager::getFallManagerEnabled(motion::IsEnabled::Request &req,
                                        motion::IsEnabled::Response &res) {
  res.is_enabled = mProxy_.getFallManagerEnabled();
  return true;
}

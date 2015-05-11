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

// Fall_Manager::Fall_Manager(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy) {
//   nh_ = nh;
//   mProxy_ = mProxy;
//   INFO("Setting up Fall Manager publishers" << std::endl);
//   robot_fallen_pub_ = nh_->advertise<std_msgs::Bool>("hasFallen", 10);
//   INFO("Setting up Fall Manager services" << std::endl);
//   srv_set_fall_manager_ = nh_->advertiseService("setFallManagerEnabled",
//                           &Fall_Manager::setFallManagerEnabled, this);
//   srv_get_fall_manager_ = nh_->advertiseService("getFallManagerEnabled",
//                           &Fall_Manager::getFallManagerEnabled, this);
//   INFO("Setting up Fall Manager subscribers" << std::endl);
//   // BIND_METHOD(Fall_Manager::hasFallen);
//   memProxy_ = new AL::ALMemoryProxy("127.0.0.1", 9559);
//   hasFallen_ = memProxy_->getData("robotHasFallen");
//   DEBUG(hasFallen_ << std::endl);
//   memProxy_->subscribeToEvent("RightBumperPressed", "Fall_Manager", "hasFallen");
// }

// Fall_Manager::~Fall_Manager() {
//   ros::shutdown();
// }

// void Fall_Manager::hasFallen() {
//   DEBUG("Test");
// }


Fall_Manager::Fall_Manager(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name): AL::ALModule(broker, name),
  fCallbackMutex(AL::ALMutex::createALMutex()) {
  setModuleDescription("This module presents how to subscribe to a simple event (here RightBumperPressed) and use a callback method.");

  functionName("onRightBumperPressed", getName(),
               "Method called when the right bumper is pressed. Makes a LED animation.");
  BIND_METHOD(Fall_Manager::onRightBumperPressed)
}

Fall_Manager::~Fall_Manager() {
  fMemoryProxy.unsubscribeToEvent("RightBumperPressed", "Fall_Manager");
}

void Fall_Manager::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());

    fState = fMemoryProxy.getData("RightBumperPressed");

    fMemoryProxy.subscribeToEvent("RightBumperPressed", "Fall_Manager",
                                  "onRightBumperPressed");
  } catch (const AL::ALError& e) {
    DEBUG(e.what() << std::endl);
  }
}

void Fall_Manager::onRightBumperPressed() {
  DEBUG("Executing callback method on right bumper event" << std::endl);
  /**
  * As long as this is defined, the code is thread-safe.
  */
  AL::ALCriticalSection section(fCallbackMutex);

  /**
  * Check that the bumper is pressed.
  */
  fState =  fMemoryProxy.getData("RightBumperPressed");
  if (fState  > 0.5f) {
    return;
  }
  try {
    DEBUG("PRESSED!");
  } catch (const AL::ALError& e) {
    DEBUG(e.what() << std::endl);
  }
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
  // DEBUG(hasFallen_ << std::endl);
  return true;
}

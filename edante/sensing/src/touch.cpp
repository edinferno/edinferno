/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Testing ALModule and ROS topic integration
*/

#include "touch.h"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <althread/alcriticalsection.h>

#include <qi/log.hpp>

Touch::Touch(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name): AL::ALModule(broker, name),
  fCallbackMutex(AL::ALMutex::createALMutex()) {
  qi::log::setVerbosity(qi::log::silent);
  // setModuleDescription("Sensing Test module.");

  functionName("rightBumperPressed", getName(), "Right bumper is pressed.");
  BIND_METHOD(Touch::rightBumperPressed)
  functionName("leftBumperPressed", getName(), "Left bumper is pressed.");
  BIND_METHOD(Touch::leftBumperPressed)
  functionName("frontHeadPressed", getName(), "Front head is pressed.");
  BIND_METHOD(Touch::frontHeadPressed)
  functionName("middleHeadPressed", getName(), "Middle head is pressed.");
  BIND_METHOD(Touch::middleHeadPressed)
  functionName("rearHeadPressed", getName(), "Rear head is pressed.");
  BIND_METHOD(Touch::rearHeadPressed)
  functionName("handRightBackPressed", getName(),
               "Hand right back is pressed.");
  BIND_METHOD(Touch::handRightBackPressed)
  functionName("handRightLeftPressed", getName(),
               "Hand right left is pressed.");
  BIND_METHOD(Touch::handRightLeftPressed)
  functionName("handRightRightPressed", getName(),
               "Hand right right is pressed.");
  BIND_METHOD(Touch::handRightRightPressed)
  functionName("handLeftBackPressed", getName(),
               "Hand left back is pressed.");
  BIND_METHOD(Touch::handLeftBackPressed)
  functionName("handLeftLeftPressed", getName(),
               "Hand left left is pressed.");
  BIND_METHOD(Touch::handLeftLeftPressed)
  functionName("handLeftRightPressed", getName(),
               "Hand left right is pressed.");
  BIND_METHOD(Touch::handLeftRightPressed)
  functionName("singleChestClick", getName(),
               "Single chest button click event.");
  BIND_METHOD(Touch::singleChestClick)
  functionName("doubleChestClick", getName(),
               "Double chest button click event.");
  BIND_METHOD(Touch::doubleChestClick)
  functionName("tripleChestClick", getName(),
               "Triple chest button click event.");
  BIND_METHOD(Touch::tripleChestClick)
}

Touch::~Touch() {
  fMemoryProxy.unsubscribeToEvent("RightBumperPressed", "Touch");
  fMemoryProxy.unsubscribeToEvent("LeftBumperPressed", "Touch");
  fMemoryProxy.unsubscribeToEvent("FrontTactilTouched", "Touch");
  fMemoryProxy.unsubscribeToEvent("MiddleTactilTouched", "Touch");
  fMemoryProxy.unsubscribeToEvent("RearTactilTouched", "Touch");
  fMemoryProxy.unsubscribeToEvent("HandRightBackTouched", "Touch");
  fMemoryProxy.unsubscribeToEvent("HandRightLeftTouched", "Touch");
  fMemoryProxy.unsubscribeToEvent("HandRightRightTouched", "Touch");
  fMemoryProxy.unsubscribeToEvent("HandLeftBackTouched", "Touch");
  fMemoryProxy.unsubscribeToEvent("HandLeftLeftTouched", "Touch");
  fMemoryProxy.unsubscribeToEvent("HandLeftRightTouched", "Touch");
  fMemoryProxy.unsubscribeToEvent("ALSentinel/SimpleClickOccured", "Touch");
  fMemoryProxy.unsubscribeToEvent("ALSentinel/DoubleClickOccured", "Touch");
  fMemoryProxy.unsubscribeToEvent("ALSentinel/TripleClickOccured", "Touch");
  // ros::shutdown();
  // delete nh_;
}

void Touch::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());
    fMemoryProxy.subscribeToEvent("RightBumperPressed", "Touch",
                                  "rightBumperPressed");
    fMemoryProxy.subscribeToEvent("LeftBumperPressed", "Touch",
                                  "leftBumperPressed");
    fMemoryProxy.subscribeToEvent("FrontTactilTouched", "Touch",
                                  "frontHeadPressed");
    fMemoryProxy.subscribeToEvent("MiddleTactilTouched", "Touch",
                                  "middleHeadPressed");
    fMemoryProxy.subscribeToEvent("RearTactilTouched", "Touch",
                                  "rearHeadPressed");
    fMemoryProxy.subscribeToEvent("HandRightBackTouched", "Touch",
                                  "handRightBackPressed");
    fMemoryProxy.subscribeToEvent("HandRightLeftTouched", "Touch",
                                  "handRightLeftPressed");
    fMemoryProxy.subscribeToEvent("HandRightRightTouched", "Touch",
                                  "handRightRightPressed");
    fMemoryProxy.subscribeToEvent("HandLeftBackTouched", "Touch",
                                  "handLeftBackPressed");
    fMemoryProxy.subscribeToEvent("HandLeftLeftTouched", "Touch",
                                  "handLeftLeftPressed");
    fMemoryProxy.subscribeToEvent("HandLeftRightTouched", "Touch",
                                  "handLeftRightPressed");
    fMemoryProxy.subscribeToEvent("ALSentinel/SimpleClickOccured", "Touch",
                                  "singleChestClick");
    fMemoryProxy.subscribeToEvent("ALSentinel/DoubleClickOccured", "Touch",
                                  "doubleChestClick");
    fMemoryProxy.subscribeToEvent("ALSentinel/TripleClickOccured", "Touch",
                                  "tripleChestClick");
  } catch (const AL::ALError& e) {
    DEBUG(e.what() << std::endl);
  }
}

void Touch::rosSetup(ros::NodeHandle* nh) {
  nh_ = nh;
  bumpers_pub_ = nh_->advertise<sensing::Bumpers>("bumpers", 10);
  chest_pub_ = nh_->advertise<std_msgs::String>("chest", 10);
  head_pub_ = nh_->advertise<sensing::Head>("head", 10);
  right_hand_pub_ = nh_->advertise<sensing::Hand>("right_hand", 10);
  left_hand_pub_ = nh_->advertise<sensing::Hand>("left_hand", 10);
}

void Touch::rightBumperPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("RightBumperPressed");
  if (fState  > 0.5f) {
    bumperMsg_.right = true;
  } else {
    bumperMsg_.right = false;
  }
  bumpers_pub_.publish(bumperMsg_);
}

void Touch::leftBumperPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("LeftBumperPressed");
  if (fState  > 0.5f) {
    bumperMsg_.left = true;
  } else {
    bumperMsg_.left = false;
  }
  bumpers_pub_.publish(bumperMsg_);
}

void Touch::singleChestClick() {
  AL::ALCriticalSection section(fCallbackMutex);
  chestMsg_.data = "single";
  chest_pub_.publish(chestMsg_);
}

void Touch::doubleChestClick() {
  AL::ALCriticalSection section(fCallbackMutex);
  chestMsg_.data = "double";
  chest_pub_.publish(chestMsg_);
}

void Touch::tripleChestClick() {
  AL::ALCriticalSection section(fCallbackMutex);
  chestMsg_.data = "triple";
  chest_pub_.publish(chestMsg_);
}

void Touch::frontHeadPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("FrontTactilTouched");
  if (fState  > 0.5f) {
    headMsg_.front = true;
  } else {
    headMsg_.front = false;
  }
  head_pub_.publish(headMsg_);
}

void Touch::middleHeadPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("MiddleTactilTouched");
  if (fState  > 0.5f) {
    headMsg_.middle = true;
  } else {
    headMsg_.middle = false;
  }
  head_pub_.publish(headMsg_);
}

void Touch::rearHeadPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("RearTactilTouched");
  if (fState  > 0.5f) {
    headMsg_.rear = true;
  } else {
    headMsg_.rear = false;
  }
  head_pub_.publish(headMsg_);
}

void Touch::handRightBackPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("HandRightBackTouched");
  if (fState  > 0.5f) {
    rightHandMsg_.back = true;
  } else {
    rightHandMsg_.back = false;
  }
  right_hand_pub_.publish(rightHandMsg_);
}

void Touch::handRightLeftPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("HandRightLeftTouched");
  if (fState  > 0.5f) {
    rightHandMsg_.left = true;
  } else {
    rightHandMsg_.left = false;
  }
  right_hand_pub_.publish(rightHandMsg_);
}

void Touch::handRightRightPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("HandRightRightTouched");
  if (fState  > 0.5f) {
    rightHandMsg_.right = true;
  } else {
    rightHandMsg_.right = false;
  }
  right_hand_pub_.publish(rightHandMsg_);
}

void Touch::handLeftBackPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("HandLeftBackTouched");
  if (fState  > 0.5f) {
    leftHandMsg_.back = true;
  } else {
    leftHandMsg_.back = false;
  }
  left_hand_pub_.publish(leftHandMsg_);
}

void Touch::handLeftLeftPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("HandLeftLeftTouched");
  if (fState  > 0.5f) {
    leftHandMsg_.left = true;
  } else {
    leftHandMsg_.left = false;
  }
  left_hand_pub_.publish(leftHandMsg_);
}

void Touch::handLeftRightPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("HandLeftRightTouched");
  if (fState  > 0.5f) {
    leftHandMsg_.right = true;
  } else {
    leftHandMsg_.right = false;
  }
  left_hand_pub_.publish(leftHandMsg_);
}


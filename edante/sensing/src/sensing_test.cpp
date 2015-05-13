/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Testing ALModule and ROS topic integration
*/

#include "sensing_test.h"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <althread/alcriticalsection.h>

#include <qi/log.hpp>

Sensing_Test::Sensing_Test(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name): AL::ALModule(broker, name),
  fCallbackMutex(AL::ALMutex::createALMutex()) {
  qi::log::setVerbosity(qi::log::silent);
  // setModuleDescription("Sensing Test module.");

  functionName("rightBumperPressed", getName(), "Right bumper is pressed.");
  BIND_METHOD(Sensing_Test::rightBumperPressed)
  functionName("leftBumperPressed", getName(), "Left bumper is pressed.");
  BIND_METHOD(Sensing_Test::leftBumperPressed)
  functionName("chestButtonPressed", getName(), "Chest button is pressed.");
  BIND_METHOD(Sensing_Test::chestButtonPressed)
  functionName("frontHeadPressed", getName(), "Front head is pressed.");
  BIND_METHOD(Sensing_Test::frontHeadPressed)
  functionName("middleHeadPressed", getName(), "Middle head is pressed.");
  BIND_METHOD(Sensing_Test::middleHeadPressed)
  functionName("rearHeadPressed", getName(), "Rear head is pressed.");
  BIND_METHOD(Sensing_Test::rearHeadPressed)
  functionName("handRightBackPressed", getName(),
               "Hand right back is pressed.");
  BIND_METHOD(Sensing_Test::handRightBackPressed)
  functionName("handRightLeftPressed", getName(),
               "Hand right left is pressed.");
  BIND_METHOD(Sensing_Test::handRightLeftPressed)
  functionName("handRightRightPressed", getName(),
               "Hand right right is pressed.");
  BIND_METHOD(Sensing_Test::handRightRightPressed)
  functionName("handLeftBackPressed", getName(),
               "Hand left back is pressed.");
  BIND_METHOD(Sensing_Test::handLeftBackPressed)
  functionName("handLeftLeftPressed", getName(),
               "Hand left left is pressed.");
  BIND_METHOD(Sensing_Test::handLeftLeftPressed)
  functionName("handLeftRightPressed", getName(),
               "Hand left right is pressed.");
  BIND_METHOD(Sensing_Test::handLeftRightPressed)
  int argc = 0;
  char *argv[] = {NULL};
  ros::init(argc, argv, "sensing_test");
  nh_ = new ros::NodeHandle("sensing_test");
  bumpers_pub_ = nh_->advertise<sensing::bumpers>("bumpers", 10);
  chest_pub_ = nh_->advertise<std_msgs::Bool>("chest", 10);
  head_pub_ = nh_->advertise<sensing::head>("head", 10);
  right_hand_pub_ = nh_->advertise<sensing::hand>("rightHand", 10);
  left_hand_pub_ = nh_->advertise<sensing::hand>("leftHand", 10);
}

Sensing_Test::~Sensing_Test() {
  fMemoryProxy.unsubscribeToEvent("RightBumperPressed", "Sensing_Test");
  fMemoryProxy.unsubscribeToEvent("LeftBumperPressed", "Sensing_Test");
  fMemoryProxy.unsubscribeToEvent("ChestButtonPressed", "Sensing_Test");
  fMemoryProxy.unsubscribeToEvent("FrontTactilTouched", "Sensing_Test");
  fMemoryProxy.unsubscribeToEvent("MiddleTactilTouched", "Sensing_Test");
  fMemoryProxy.unsubscribeToEvent("RearTactilTouched", "Sensing_Test");
  fMemoryProxy.unsubscribeToEvent("HandRightBackTouched", "Sensing_Test");
  fMemoryProxy.unsubscribeToEvent("HandRightLeftTouched", "Sensing_Test");
  fMemoryProxy.unsubscribeToEvent("HandRightRightTouched", "Sensing_Test");
  fMemoryProxy.unsubscribeToEvent("HandLeftBackTouched", "Sensing_Test");
  fMemoryProxy.unsubscribeToEvent("HandLeftLeftTouched", "Sensing_Test");
  fMemoryProxy.unsubscribeToEvent("HandLeftRightTouched", "Sensing_Test");
  ros::shutdown();
  delete nh_;
}

void Sensing_Test::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());
    fMemoryProxy.subscribeToEvent("RightBumperPressed", "Sensing_Test",
                                  "rightBumperPressed");
    fMemoryProxy.subscribeToEvent("LeftBumperPressed", "Sensing_Test",
                                  "leftBumperPressed");
    fMemoryProxy.subscribeToEvent("ChestButtonPressed", "Sensing_Test",
                                  "chestButtonPressed");
    fMemoryProxy.subscribeToEvent("FrontTactilTouched", "Sensing_Test",
                                  "frontHeadPressed");
    fMemoryProxy.subscribeToEvent("MiddleTactilTouched", "Sensing_Test",
                                  "middleHeadPressed");
    fMemoryProxy.subscribeToEvent("RearTactilTouched", "Sensing_Test",
                                  "rearHeadPressed");
    fMemoryProxy.subscribeToEvent("HandRightBackTouched", "Sensing_Test",
                                  "handRightBackPressed");
    fMemoryProxy.subscribeToEvent("HandRightLeftTouched", "Sensing_Test",
                                  "handRightLeftPressed");
    fMemoryProxy.subscribeToEvent("HandRightRightTouched", "Sensing_Test",
                                  "handRightRightPressed");
    fMemoryProxy.subscribeToEvent("HandLeftBackTouched", "Sensing_Test",
                                  "handLeftBackPressed");
    fMemoryProxy.subscribeToEvent("HandLeftLeftTouched", "Sensing_Test",
                                  "handLeftLeftPressed");
    fMemoryProxy.subscribeToEvent("HandLeftRightTouched", "Sensing_Test",
                                  "handLeftRightPressed");
  } catch (const AL::ALError& e) {
    DEBUG(e.what() << std::endl);
  }
}

void Sensing_Test::rightBumperPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("RightBumperPressed");
  if (fState  > 0.5f) {
    bumperMsg_.right = true;
  } else {
    bumperMsg_.right = false;
  }
  bumpers_pub_.publish(bumperMsg_);
}

void Sensing_Test::leftBumperPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("LeftBumperPressed");
  if (fState  > 0.5f) {
    bumperMsg_.left = true;
  } else {
    bumperMsg_.left = false;
  }
  bumpers_pub_.publish(bumperMsg_);
}

void Sensing_Test::chestButtonPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  std_msgs::Bool msg;
  fState =  fMemoryProxy.getData("ChestButtonPressed");
  if (fState  > 0.5f) {
    msg.data = true;
  } else {
    msg.data = false;
  }
  chest_pub_.publish(msg);
}

void Sensing_Test::frontHeadPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("FrontTactilTouched");
  if (fState  > 0.5f) {
    headMsg_.front = true;
  } else {
    headMsg_.front = false;
  }
  head_pub_.publish(headMsg_);
}

void Sensing_Test::middleHeadPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("MiddleTactilTouched");
  if (fState  > 0.5f) {
    headMsg_.middle = true;
  } else {
    headMsg_.middle = false;
  }
  head_pub_.publish(headMsg_);
}

void Sensing_Test::rearHeadPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("RearTactilTouched");
  if (fState  > 0.5f) {
    headMsg_.rear = true;
  } else {
    headMsg_.rear = false;
  }
  head_pub_.publish(headMsg_);
}

void Sensing_Test::handRightBackPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("HandRightBackTouched");
  if (fState  > 0.5f) {
    rightHandMsg_.back = true;
  } else {
    rightHandMsg_.back = false;
  }
  right_hand_pub_.publish(rightHandMsg_);
}

void Sensing_Test::handRightLeftPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("HandRightLeftTouched");
  if (fState  > 0.5f) {
    rightHandMsg_.left = true;
  } else {
    rightHandMsg_.left = false;
  }
  right_hand_pub_.publish(rightHandMsg_);
}

void Sensing_Test::handRightRightPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("HandRightRightTouched");
  if (fState  > 0.5f) {
    rightHandMsg_.right = true;
  } else {
    rightHandMsg_.right = false;
  }
  right_hand_pub_.publish(rightHandMsg_);
}

void Sensing_Test::handLeftBackPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("HandLeftBackTouched");
  if (fState  > 0.5f) {
    leftHandMsg_.back = true;
  } else {
    leftHandMsg_.back = false;
  }
  left_hand_pub_.publish(leftHandMsg_);
}

void Sensing_Test::handLeftLeftPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("HandLeftLeftTouched");
  if (fState  > 0.5f) {
    leftHandMsg_.left = true;
  } else {
    leftHandMsg_.left = false;
  }
  left_hand_pub_.publish(leftHandMsg_);
}

void Sensing_Test::handLeftRightPressed() {
  AL::ALCriticalSection section(fCallbackMutex);
  fState =  fMemoryProxy.getData("HandLeftRightTouched");
  if (fState  > 0.5f) {
    leftHandMsg_.right = true;
  } else {
    leftHandMsg_.right = false;
  }
  left_hand_pub_.publish(leftHandMsg_);
}

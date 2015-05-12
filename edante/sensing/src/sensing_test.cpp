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
  setModuleDescription("Sensing Test module.");

  functionName("onRightBumperPressed", getName(),
               "Method called when the right bumper is pressed.");
  BIND_METHOD(Sensing_Test::onRightBumperPressed)
  int argc = 0;
  char *argv[] = {NULL};
  ros::init(argc, argv, "sensing_test");
  nh_ = new ros::NodeHandle("sensing_test");
  robot_fallen_pub_ = nh_->advertise<std_msgs::Bool>("hasFallen", 10);
}

Sensing_Test::~Sensing_Test() {
  fMemoryProxy.unsubscribeToEvent("RightBumperPressed", "Sensing_Test");
  ros::shutdown();
  delete nh_;
}

void Sensing_Test::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());

    fState = fMemoryProxy.getData("RightBumperPressed");

    fMemoryProxy.subscribeToEvent("RightBumperPressed", "Sensing_Test",
                                  "onRightBumperPressed");
  } catch (const AL::ALError& e) {
    DEBUG(e.what() << std::endl);
  }
}

void Sensing_Test::onRightBumperPressed() {
  /**
  * As long as this is defined, the code is thread-safe.
  */
  AL::ALCriticalSection section(fCallbackMutex);
  std_msgs::Bool msg;
  fState =  fMemoryProxy.getData("RightBumperPressed");
  if (fState  > 0.5f) {
    msg.data = true;
    robot_fallen_pub_.publish(msg);
  } else {
    msg.data = false;
    robot_fallen_pub_.publish(msg);
  }
}

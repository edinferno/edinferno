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

Sensing::Sensing(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name): AL::ALModule(broker, name),
  fCallbackMutex(AL::ALMutex::createALMutex()) {
  setModuleDescription("This module presents how to subscribe to a simple event (here RightBumperPressed) and use a callback method.");

  functionName("onRightBumperPressed", getName(),
               "Method called when the right bumper is pressed. Makes a LED animation.");
  BIND_METHOD(Sensing::onRightBumperPressed)
}

Sensing::~Sensing() {
  fMemoryProxy.unsubscribeToEvent("RightBumperPressed", "Sensing");
}

void Sensing::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());

    fState = fMemoryProxy.getData("RightBumperPressed");

    fMemoryProxy.subscribeToEvent("RightBumperPressed", "Sensing",
                                  "onRightBumperPressed");
  } catch (const AL::ALError& e) {
    DEBUG(e.what() << std::endl);
  }
}

void Sensing::onRightBumperPressed() {
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

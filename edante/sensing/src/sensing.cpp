/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-11
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS Wrapper for NaoQI sensor events
*/

#include <ros/ros.h>
#include "definitions.h"

#include "touch.h"
#include "power.h"
#include "sonar.h"
#include "fsr.h"

#include <alproxies/almemoryproxy.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "sensing");
  ros::NodeHandle nh("sensing");
  AL::ALMemoryProxy memProxy("127.0.0.1", 9559);
  // Touch TouchTest(&nh, &memProxy);
  // Power PowerTest(&nh, &memProxy);
  // Sonar SonarTest(&nh, &memProxy);
  // Fsr FsrTest(&nh, &memProxy);

  ros::Rate r(20);

  while (ros::ok()) {
    // TouchTest.spinTopics();
    // PowerTest.spinTopics();
    // SonarTest.spinTopics();
    // FsrTest.spinTopics();
    ros::spinOnce();
    r.sleep();
  }

}

// #include "sensing.h"

// #include <alvalue/alvalue.h>
// #include <alcommon/alproxy.h>
// #include <alcommon/albroker.h>
// #include <althread/alcriticalsection.h>

// Sensing::Sensing(
//   boost::shared_ptr<AL::ALBroker> broker,
//   const std::string& name): AL::ALModule(broker, name),
//   fCallbackMutex(AL::ALMutex::createALMutex()) {
//   setModuleDescription("This module presents how to subscribe to a simple event (here RightBumperPressed) and use a callback method.");

//   functionName("onRightBumperPressed", getName(),
//                "Method called when the right bumper is pressed. Makes a LED animation.");
//   BIND_METHOD(Sensing::onRightBumperPressed)
// }

// Sensing::~Sensing() {
//   fMemoryProxy.unsubscribeToEvent("RightBumperPressed", "Sensing");
// }

// void Sensing::init() {
//   try {
//     fMemoryProxy = AL::ALMemoryProxy(getParentBroker());

//     fState = fMemoryProxy.getData("RightBumperPressed");

//     fMemoryProxy.subscribeToEvent("RightBumperPressed", "Sensing",
//                                   "onRightBumperPressed");
//   } catch (const AL::ALError& e) {
//     DEBUG(e.what() << std::endl);
//   }
// }

// void Sensing::onRightBumperPressed() {
//   DEBUG("Executing callback method on right bumper event" << std::endl);
//   /**
//   * As long as this is defined, the code is thread-safe.
//   */
//   AL::ALCriticalSection section(fCallbackMutex);

//   /**
//   * Check that the bumper is pressed.
//   */
//   fState =  fMemoryProxy.getData("RightBumperPressed");
//   if (fState  > 0.5f) {
//     return;
//   }
//   try {
//     DEBUG("PRESSED!");
//   } catch (const AL::ALError& e) {
//     DEBUG(e.what() << std::endl);
//   }
// }

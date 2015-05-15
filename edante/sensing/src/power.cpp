/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's power sensors
*/

#include "power.h"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <althread/alcriticalsection.h>

#include <qi/log.hpp>

Power::Power(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name): AL::ALModule(broker, name),
  fCallbackMutex(AL::ALMutex::createALMutex()) {
  qi::log::setVerbosity(qi::log::info);
  // setModuleDescription("Sensing Test module.");
  functionName("powerPub", getName(), "Plugged / Unplugged + battery charge");
  BIND_METHOD(Power::powerPub)
}

Power::~Power() {
  fMemoryProxy.unsubscribeToEvent("BatteryPowerPluggedChanged", "Power");
  fMemoryProxy.unsubscribeToEvent("BatteryChargeChanged", "Power");
}

void Power::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());
    fMemoryProxy.subscribeToEvent("BatteryPowerPluggedChanged", "Power",
                                  "powerPub");
    fMemoryProxy.subscribeToEvent("BatteryChargeChanged", "Power",
                                  "powerPub");
  } catch (const AL::ALError& e) {
    DEBUG(e.what() << std::endl);
  }
}

void Power::rosSetup(ros::NodeHandle * nh) {
  nh_ = nh;
  power_status_pub_ = nh_->advertise<std_msgs::String>("powerStatus", 10);
  power_charge_pub_ = nh_->advertise<std_msgs::UInt32>("powerCharge", 10);
}

void Power::powerPub() {
  AL::ALCriticalSection section(fCallbackMutex);
  bool b = fMemoryProxy.getData("BatteryPowerPluggedChanged");
  if (b) {powerEvent.data = "plugged";} else {powerEvent.data = "unplugged";}
  power_status_pub_.publish(powerEvent);
  powerCharge.data = int(fMemoryProxy.getData("BatteryChargeChanged"));
  power_charge_pub_.publish(powerCharge);
}

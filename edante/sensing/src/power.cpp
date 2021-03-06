/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's power sensors
*/

#include "sensing/power.hpp"

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
  functionName("hotJointDetected", getName(), "Hot Joint event callback");
  BIND_METHOD(Power::hotJointDetected)
}

Power::~Power() {
  fMemoryProxy.unsubscribeToEvent("BatteryPowerPluggedChanged", "Power");
  fMemoryProxy.unsubscribeToEvent("BatteryChargeChanged", "Power");
  fMemoryProxy.unsubscribeToEvent("HotJointDetected", "Power");
}

void Power::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());
    fMemoryProxy.subscribeToEvent("BatteryPowerPluggedChanged", "Power",
                                  "powerPub");
    fMemoryProxy.subscribeToEvent("BatteryChargeChanged", "Power",
                                  "powerPub");
    fMemoryProxy.subscribeToEvent("HotJointDetected", "Power",
                                  "hotJointDetected");
  } catch (const AL::ALError& e) {
    ROS_DEBUG_STREAM(e.what());
  }
}

void Power::rosSetup(ros::NodeHandle* nh) {
  nh_ = nh;
  power_status_pub_ = nh_->advertise<std_msgs::String>("power_status", 10);
  power_charge_pub_ = nh_->advertise<std_msgs::Int32>("power_charge", 10);
}

void Power::powerPub() {
  AL::ALCriticalSection section(fCallbackMutex);
  bool b = fMemoryProxy.getData("BatteryPowerPluggedChanged");
  if (b) {powerEvent.data = "plugged";} else {powerEvent.data = "unplugged";}
  power_status_pub_.publish(powerEvent);
  powerCharge.data = static_cast<int>
                     (fMemoryProxy.getData("BatteryChargeChanged"));
  power_charge_pub_.publish(powerCharge);
}

void Power::hotJointDetected() {
  AL::ALCriticalSection section(fCallbackMutex);
  ROS_WARN("Hot Joint Detected!");
}

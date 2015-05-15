/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's sonar sensors
*/

#include "sonar.h"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <althread/alcriticalsection.h>

#include <qi/log.hpp>

Sonar::Sonar(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name): AL::ALModule(broker, name),
  fCallbackMutex(AL::ALMutex::createALMutex()) {
  qi::log::setVerbosity(qi::log::info);
  // setModuleDescription("Sensing Test module.");

  sonarProxy_ = new AL::ALSonarProxy("127.0.0.1", 9559);

  sonarProxy_->subscribe("Sonar");

  functionName("sonarLeftDetected", getName(), "Left sonar obstacle");
  BIND_METHOD(Sonar::sonarLeftDetected)
  functionName("sonarRightDetected", getName(), "Right sonar obstacle");
  BIND_METHOD(Sonar::sonarRightDetected)
  functionName("sonarLeftNothingDetected", getName(), "No left sonar obstacle");
  BIND_METHOD(Sonar::sonarLeftNothingDetected)
  functionName("sonarRightNothingDetected", getName(), "No right sonar obstacle");
  BIND_METHOD(Sonar::sonarRightNothingDetected)
}

Sonar::~Sonar() {
  fMemoryProxy.unsubscribeToEvent("SonarLeftDetected", "Sonar");
  fMemoryProxy.unsubscribeToEvent("SonarRightDetected", "Sonar");
  fMemoryProxy.unsubscribeToEvent("SonarLeftNothingDetected", "Sonar");
  fMemoryProxy.unsubscribeToEvent("SonarRightNothingDetected", "Sonar");
  ros::shutdown();
}

void Sonar::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());
    fMemoryProxy.subscribeToEvent("SonarLeftDetected", "Sonar",
                                  "sonarLeftDetected");
    fMemoryProxy.subscribeToEvent("SonarRightDetected", "Sonar",
                                  "sonarRightDetected");
    fMemoryProxy.subscribeToEvent("SonarLeftNothingDetected", "Sonar",
                                  "sonarLeftNothingDetected");
    fMemoryProxy.subscribeToEvent("SonarRightNothingDetected", "Sonar",
                                  "sonarRightNothingDetected");
  } catch (const AL::ALError& e) {
    DEBUG(e.what() << std::endl);
  }
}

void Sonar::rosSetup(ros::NodeHandle* nh, bool pubSonars) {
  nh_ = nh;
  pubSonars_ = pubSonars;
  sonar_event_pub_ = nh_->advertise<std_msgs::String>("sonarEvent", 10);
  sonar_data_pub_ = nh_->advertise<sensing::sonars>("sonarData", 10);
  srv_enab_sonar_ = nh_->advertiseService("enableSonarPub",
                                          &Sonar::enableSonarPub, this);
}

void Sonar::spin() {
  if (pubSonars_) {
    this->pubSonars();
  }
}

void Sonar::sonarLeftDetected () {
  std_msgs::String event;
  event.data = "left";
  sonar_event_pub_.publish(event);
  this->pubSonars();
}

void Sonar::sonarRightDetected () {
  std_msgs::String event;
  event.data = "right";
  sonar_event_pub_.publish(event);
  this->pubSonars();
}

void Sonar::sonarLeftNothingDetected () {
  std_msgs::String event;
  event.data = "noleft";
  sonar_event_pub_.publish(event);
  this->pubSonars();
}

void Sonar::sonarRightNothingDetected () {
  std_msgs::String event;
  event.data = "noright";
  sonar_event_pub_.publish(event);
  this->pubSonars();
}

void Sonar::pubSonars() {
  AL::ALCriticalSection section(fCallbackMutex);
  sensing::sonars data;
  data.left = fMemoryProxy.getData("Device/SubDeviceList/US/Left/Sensor/Value");
  data.right = fMemoryProxy.getData("Device/SubDeviceList/US/Right/Sensor/Value");
  sonar_data_pub_.publish(data);
}

bool Sonar::enableSonarPub(sensing::enable::Request &req,
                           sensing::enable::Response &res) {
  pubSonars_ = req.isEnabled;
  return true;
}

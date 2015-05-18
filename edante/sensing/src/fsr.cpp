/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's fsr sensors
*/

#include "fsr.h"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <althread/alcriticalsection.h>

#include <qi/log.hpp>

Fsr::Fsr(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name): AL::ALModule(broker, name),
  fCallbackMutex(AL::ALMutex::createALMutex()) {
  qi::log::setVerbosity(qi::log::info);
  // setModuleDescription("Sensing Test module.");

  functionName("pubContact", getName(), "Fsr publisher");
  BIND_METHOD(Fsr::pubContact)
}

Fsr::~Fsr() {
  fMemoryProxy.unsubscribeToEvent("footContactChanged", "Fsr");
}

void Fsr::init() {
  try {
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());
    fMemoryProxy.subscribeToEvent("footContactChanged", "Fsr",
                                  "pubContact");
  } catch (const AL::ALError& e) {
    DEBUG(e.what() << std::endl);
  }
}

void Fsr::rosSetup(ros::NodeHandle* nh, bool pubFsrs) {
  nh_ = nh;
  pubFsrs_ = pubFsrs;
  fsr_contact_pub_ = nh_->advertise<std_msgs::Bool>("fsrContact", 10);
  fsr_data_pub_ = nh_->advertise<sensing::fsr>("fsrData", 10);
  srv_enab_fsr_ = nh_->advertiseService("enableFsrPub",
                                        &Fsr::enableFsrPub, this);
}

void Fsr::spin() {
  if (pubFsrs_) {
    this->pubFsr();
  }
}

void Fsr::pubContact() {
  AL::ALCriticalSection section(fCallbackMutex);
  std_msgs::Bool msg;
  msg.data = static_cast<bool>(fMemoryProxy.getData("footContactChanged"));
  fsr_contact_pub_.publish(msg);
}

void Fsr::pubFsr() {
  AL::ALCriticalSection section(fCallbackMutex);
  sensing::fsr msg;
  if (static_cast<float>(fMemoryProxy.getData("leftFootContact")) > 0.5f)
  {msg.leftContact = true;} else {msg.leftContact = false;}
  if (static_cast<float>(fMemoryProxy.getData("rightFootContact")) > 0.5f)
  {msg.rightContact = true;} else {msg.rightContact = false;}
  msg.leftWeight = static_cast<float>
                   (fMemoryProxy.getData("leftFootTotalWeight"));
  msg.rightWeight = static_cast<float>
                    (fMemoryProxy.getData("rightFootTotalWeight"));
  fsr_data_pub_.publish(msg);
}

bool Fsr::enableFsrPub(sensing::enable::Request &req,
                       sensing::enable::Response &res) {
  pubFsrs_ = req.isEnabled;
  return true;
}

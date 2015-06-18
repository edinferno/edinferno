/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's fsr sensors
*/

#include "sensing/fsr.hpp"

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
    ROS_ERROR_STREAM(e.what());
  }
}

void Fsr::rosSetup(ros::NodeHandle* nh, bool pubFsrs) {
  nh_ = nh;
  pubFsrs_ = pubFsrs;
  fsr_contact_pub_ = nh_->advertise<std_msgs::Bool>("fsr_contact", 10);
  fsr_data_pub_ = nh_->advertise<sensing_msgs::Fsr>("fsr_data", 10);
  srv_enab_fsr_ = nh_->advertiseService("enable_fsr_pub",
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
  sensing_msgs::Fsr msg;
  if (static_cast<float>(fMemoryProxy.getData("leftFootContact")) > 0.5f)
  {msg.left_contact = true;} else {msg.left_contact = false;}
  if (static_cast<float>(fMemoryProxy.getData("rightFootContact")) > 0.5f)
  {msg.right_contact = true;} else {msg.right_contact = false;}
  msg.left_weight = static_cast<float>
                    (fMemoryProxy.getData("leftFootTotalWeight"));
  msg.right_weight = static_cast<float>
                     (fMemoryProxy.getData("rightFootTotalWeight"));
  fsr_data_pub_.publish(msg);
}

bool Fsr::enableFsrPub(sensing_msgs::Enable::Request& req,
                       sensing_msgs::Enable::Response& res) {
  pubFsrs_ = req.is_enabled;
  return true;
}

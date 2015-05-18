/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's LED class
*/

#include "led.h"

Led::Led(ros::NodeHandle* nh) {
  nh_ = nh;
  leds = new AL::ALLedsProxy("127.0.0.1", 9559);
  INFO("Setting up LED signalling services" << std::endl);
  srv_create_group_ = nh_->advertiseService("createLedGroup",
                      &Led::createLedGroup, this);
  srv_led_angle_ = nh_->advertiseService("earLedsSetAngle",
                                         &Led::earLedsSetAngle, this);
  srv_fade_ = nh_->advertiseService("fade",
                                    &Led::fade, this);
  srv_list_groups_ = nh_->advertiseService("listGroups",
                     &Led::listGroups, this);
  srv_list_leds_ = nh_->advertiseService("listLEDs",
                                         &Led::listLEDs, this);
  srv_rotate_eyes_ = nh_->advertiseService("rotateEyes",
                     &Led::rotateEyes, this);
  srv_set_intensity_ = nh_->advertiseService("setIntensity",
                       &Led::setIntensity, this);
}

Led::~Led() {
}

bool Led::createLedGroup(signalling::createLedGroup::Request &req,
                         signalling::createLedGroup::Response &res) {
  leds->createGroup(req.groupName, req.ledNames);
  return true;
}

bool Led::earLedsSetAngle(signalling::earLedsSetAngle::Request &req,
                          signalling::earLedsSetAngle::Response &res) {
  leds->earLedsSetAngle(req.degrees, req.duration, req.leaveOnAtEnd);
  return true;
}

bool Led::fade(signalling::fade::Request &req,
               signalling::fade::Response &res) {
  leds->fade(req.name, req.intensity, req.duration);
  return true;
}

bool Led::listGroups(signalling::getNames::Request &req,
                     signalling::getNames::Response &res) {
  res.names = leds->listGroups();
  return true;
}

bool Led::listLEDs(signalling::getNames::Request &req,
                   signalling::getNames::Response &res) {
  res.names = leds->listLEDs();
  return true;
}

bool Led::rotateEyes(signalling::rotateEyes::Request &req,
                     signalling::rotateEyes::Response &res) {
  leds->rotateEyes(req.rgb, req.timeForRotation, req.totalDuration);
  return true;
}

bool Led::setIntensity(signalling::setIntensity::Request &req,
                       signalling::setIntensity::Response &res) {
  leds->setIntensity(req.name, req.intensity);
  return true;
}

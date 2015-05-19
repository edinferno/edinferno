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
  srv_create_group_ = nh_->advertiseService("create_led_group",
                      &Led::createLedGroup, this);
  srv_led_angle_ = nh_->advertiseService("ear_leds_set_angle",
                                         &Led::earLedsSetAngle, this);
  srv_fade_ = nh_->advertiseService("fade",
                                    &Led::fade, this);
  srv_list_groups_ = nh_->advertiseService("list_groups",
                     &Led::listGroups, this);
  srv_list_leds_ = nh_->advertiseService("list_leds",
                                         &Led::listLEDs, this);
  srv_rotate_eyes_ = nh_->advertiseService("rotate_eyes",
                     &Led::rotateEyes, this);
  srv_set_intensity_ = nh_->advertiseService("set_intensity",
                       &Led::setIntensity, this);
}

Led::~Led() {
}

bool Led::createLedGroup(signalling::CreateLedGroup::Request &req,
                         signalling::CreateLedGroup::Response &res) {
  leds->createGroup(req.group_name, req.led_names);
  return true;
}

bool Led::earLedsSetAngle(signalling::EarLedsSetAngle::Request &req,
                          signalling::EarLedsSetAngle::Response &res) {
  leds->earLedsSetAngle(req.degrees, req.duration, req.leave_on_at_end);
  return true;
}

bool Led::fade(signalling::Fade::Request &req,
               signalling::Fade::Response &res) {
  leds->fade(req.name, req.intensity, req.duration);
  return true;
}

bool Led::listGroups(signalling::GetNames::Request &req,
                     signalling::GetNames::Response &res) {
  res.names = leds->listGroups();
  return true;
}

bool Led::listLEDs(signalling::GetNames::Request &req,
                   signalling::GetNames::Response &res) {
  res.names = leds->listLEDs();
  return true;
}

bool Led::rotateEyes(signalling::RotateEyes::Request &req,
                     signalling::RotateEyes::Response &res) {
  leds->rotateEyes(req.rgb, req.time_for_rotation, req.total_duration);
  return true;
}

bool Led::setIntensity(signalling::SetIntensity::Request &req,
                       signalling::SetIntensity::Response &res) {
  leds->setIntensity(req.name, req.intensity);
  return true;
}

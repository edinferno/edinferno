/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's LED class
*/

#include "signalling/led.hpp"

Led::Led(ros::NodeHandle* nh) {
  nh_ = nh;
  leds_ = new AL::ALLedsProxy("127.0.0.1", 9559);
  ROS_INFO_STREAM("Setting up LED signalling services");
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

bool Led::createLedGroup(signalling_msgs::CreateLedGroup::Request& req,
                         signalling_msgs::CreateLedGroup::Response& res) {
  leds_->createGroup(req.group_name, req.led_names);
  return true;
}

bool Led::earLedsSetAngle(signalling_msgs::EarLedsSetAngle::Request& req,
                          signalling_msgs::EarLedsSetAngle::Response& res) {
  leds_->earLedsSetAngle(req.degrees, req.duration, req.leave_on_at_end);
  return true;
}

bool Led::fade(signalling_msgs::Fade::Request& req,
               signalling_msgs::Fade::Response& res) {
  leds_->fade(req.name, req.intensity, req.duration);
  return true;
}

bool Led::listGroups(signalling_msgs::GetNames::Request& req,
                     signalling_msgs::GetNames::Response& res) {
  res.names = leds_->listGroups();
  return true;
}

bool Led::listLEDs(signalling_msgs::GetNames::Request& req,
                   signalling_msgs::GetNames::Response& res) {
  res.names = leds_->listLEDs();
  return true;
}

bool Led::rotateEyes(signalling_msgs::RotateEyes::Request& req,
                     signalling_msgs::RotateEyes::Response& res) {
  leds_->rotateEyes(req.rgb, req.time_for_rotation, req.total_duration);
  return true;
}

bool Led::setIntensity(signalling_msgs::SetIntensity::Request& req,
                       signalling_msgs::SetIntensity::Response& res) {
  leds_->setIntensity(req.name, req.intensity);
  return true;
}

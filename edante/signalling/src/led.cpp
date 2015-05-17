/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's LED class
*/

#include "led.h"

using namespace std;

Led::Led(ros::NodeHandle* nh) {
  nh_ = nh;
  leds = new AL::ALLedsProxy("127.0.0.1", 9559);
  INFO("Setting up LED signalling services" << std::endl);
  srv_led_angle_ = nh_->advertiseService("earLedsSetAngle",
                                         &Led::earLedsSetAngle, this);
}

Led::~Led() {
}

bool Led::earLedsSetAngle(signalling::earLedsSetAngle::Request &req,
                          signalling::earLedsSetAngle::Response &res) {
  leds->earLedsSetAngle(req.degrees, req.duration, req.leaveOnAtEnd);
  return true;
}

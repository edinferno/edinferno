/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's LED class
*/

#include <ros/ros.h>

#include "signalling/createLedGroup.h"
#include "signalling/earLedsSetAngle.h"
#include "signalling/fade.h"
#include "signalling/getNames.h"
#include "signalling/rotateEyes.h"
#include "signalling/setIntensity.h"

#include <alproxies/alledsproxy.h>

#include "definitions.h"

#ifndef LED_H_
#define LED_H_

class Led {
 public:
  Led(ros::NodeHandle* nh);
  ~Led();

  bool createLedGroup(signalling::createLedGroup::Request &req,
                      signalling::createLedGroup::Response &res);
  bool earLedsSetAngle(signalling::earLedsSetAngle::Request &req,
                       signalling::earLedsSetAngle::Response &res);
  bool fade(signalling::fade::Request &req,
            signalling::fade::Response &res);
  bool listGroups(signalling::getNames::Request &req,
                  signalling::getNames::Response &res);
  bool listLEDs(signalling::getNames::Request &req,
                signalling::getNames::Response &res);
  bool rotateEyes(signalling::rotateEyes::Request &req,
                  signalling::rotateEyes::Response &res);
  bool setIntensity(signalling::setIntensity::Request &req,
                    signalling::setIntensity::Response &res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_create_group_;
  ros::ServiceServer srv_led_angle_;
  ros::ServiceServer srv_fade_;
  ros::ServiceServer srv_list_groups_;
  ros::ServiceServer srv_list_leds_;
  ros::ServiceServer srv_rotate_eyes_;
  ros::ServiceServer srv_set_intensity_;

  // NaoQI
  AL::ALLedsProxy* leds;

};

#endif /* LED_H_ */
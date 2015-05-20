/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's LED class
*/

#include <ros/ros.h>

#include <alproxies/alledsproxy.h>

#include "signalling/CreateLedGroup.h"
#include "signalling/EarLedsSetAngle.h"
#include "signalling/Fade.h"
#include "signalling/GetNames.h"
#include "signalling/RotateEyes.h"
#include "signalling/SetIntensity.h"
#include "definitions.h"

#ifndef LED_H_
#define LED_H_

class Led {
 public:
  explicit Led(ros::NodeHandle* nh);
  ~Led();

  bool createLedGroup(signalling::CreateLedGroup::Request &req,
                      signalling::CreateLedGroup::Response &res);
  bool earLedsSetAngle(signalling::EarLedsSetAngle::Request &req,
                       signalling::EarLedsSetAngle::Response &res);
  bool fade(signalling::Fade::Request &req,
            signalling::Fade::Response &res);
  bool listGroups(signalling::GetNames::Request &req,
                  signalling::GetNames::Response &res);
  bool listLEDs(signalling::GetNames::Request &req,
                signalling::GetNames::Response &res);
  bool rotateEyes(signalling::RotateEyes::Request &req,
                  signalling::RotateEyes::Response &res);
  bool setIntensity(signalling::SetIntensity::Request &req,
                    signalling::SetIntensity::Response &res);

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

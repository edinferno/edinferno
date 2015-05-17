/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's LED class
*/

#include <ros/ros.h>

#include "signalling/earLedsSetAngle.h"
#include "signalling/setIntensity.h"

#include <alproxies/alledsproxy.h>

#include "definitions.h"

#ifndef LED_H_
#define LED_H_

class Led {
 public:
  Led(ros::NodeHandle* nh);
  ~Led();

  bool earLedsSetAngle(signalling::earLedsSetAngle::Request &req,
                       signalling::earLedsSetAngle::Response &res);
  bool setIntensity(signalling::setIntensity::Request &req,
                    signalling::setIntensity::Response &res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_led_angle_;
  ros::ServiceServer srv_set_intensity_;

  // NaoQI
  AL::ALLedsProxy* leds;

};

#endif /* LED_H_ */

/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's LED class
*/

#ifndef LED_HPP
#define LED_HPP

#include <ros/ros.h>

#include <alproxies/alledsproxy.h>

#include <signalling_msgs/CreateLedGroup.h>
#include <signalling_msgs/EarLedsSetAngle.h>
#include <signalling_msgs/Fade.h>
#include <signalling_msgs/FadeRGB.h>
#include <signalling_msgs/GetNames.h>
#include <signalling_msgs/RotateEyes.h>
#include <signalling_msgs/SetIntensity.h>

class Led {
 public:
  explicit Led(ros::NodeHandle* nh);
  ~Led();

  bool createLedGroup(signalling_msgs::CreateLedGroup::Request& req,
                      signalling_msgs::CreateLedGroup::Response& res);
  bool earLedsSetAngle(signalling_msgs::EarLedsSetAngle::Request& req,
                       signalling_msgs::EarLedsSetAngle::Response& res);
  bool fade(signalling_msgs::Fade::Request& req,
            signalling_msgs::Fade::Response& res);
  bool fadeRGB(signalling_msgs::FadeRGB::Request& req,
               signalling_msgs::FadeRGB::Response& res);
  bool listGroups(signalling_msgs::GetNames::Request& req,
                  signalling_msgs::GetNames::Response& res);
  bool listLEDs(signalling_msgs::GetNames::Request& req,
                signalling_msgs::GetNames::Response& res);
  bool rotateEyes(signalling_msgs::RotateEyes::Request& req,
                  signalling_msgs::RotateEyes::Response& res);
  bool setIntensity(signalling_msgs::SetIntensity::Request& req,
                    signalling_msgs::SetIntensity::Response& res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_create_group_;
  ros::ServiceServer srv_led_angle_;
  ros::ServiceServer srv_fade_;
  ros::ServiceServer srv_fade_rgb_;
  ros::ServiceServer srv_list_groups_;
  ros::ServiceServer srv_list_leds_;
  ros::ServiceServer srv_rotate_eyes_;
  ros::ServiceServer srv_set_intensity_;

  // NaoQI
  AL::ALLedsProxy* leds_;
};

#endif /* LED_HPP_ */

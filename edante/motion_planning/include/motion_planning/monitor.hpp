/**
 * @file      monitor.hpp
 * @brief     Monitor class, leds and audio are used for external debugging
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-17
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef MONITOR_HPP
#define MONITOR_HPP

#include <ros/ros.h>

#include <signalling_msgs/CreateLedGroup.h>
#include <signalling_msgs/EarLedsSetAngle.h>
#include <signalling_msgs/Fade.h>
#include <signalling_msgs/FadeRGB.h>
#include <signalling_msgs/GetNames.h>
#include <signalling_msgs/RotateEyes.h>
#include <signalling_msgs/SetIntensity.h>
#include <signalling_msgs/SayText.h>

#include <motion_planning_msgs/MonitorMode.h>

#include <signalling_msgs/signalling_values.hpp>

class Monitor {
 public:
  explicit Monitor(ros::NodeHandle* nh);
  ~Monitor();

  void init();

  bool setMonitorMode(motion_planning_msgs::MonitorMode::Request& req,
                      motion_planning_msgs::MonitorMode::Response& res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_monitor_mode_;
  ros::ServiceClient create_led_group_;
  ros::ServiceClient ear_leds_angle_;
  ros::ServiceClient fade_led_;
  ros::ServiceClient fade_rgb_led_;
  signalling_msgs::FadeRGB right_eye_leds_;
  ros::ServiceClient rotate_eyes_;
  ros::ServiceClient set_intensity_led_;
  signalling_msgs::SetIntensity left_eye_top_;
  signalling_msgs::SetIntensity left_eye_bottom_;
  signalling_msgs::SetIntensity face_leds_;
  ros::ServiceClient speak_text_;
};

#endif /* MONITOR_HPP_ */

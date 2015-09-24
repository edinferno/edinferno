/**
 * @file      monitor.cpp
 * @brief     Monitor class, leds and audio are used for external debugging
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-17
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion_planning/monitor.hpp"

Monitor::Monitor(ros::NodeHandle* nh) {
  nh_ = nh;
  ROS_INFO_STREAM("Setting up Monitoring services");
  srv_monitor_mode_ = nh_->advertiseService("monitor_mode",
                                            &Monitor::setMonitorMode, this);
  ros::service::waitForService("/signalling/create_led_group");
  create_led_group_ = nh_->serviceClient<signalling_msgs::CreateLedGroup>(
                        "/signalling/create_led_group", true);
  ros::service::waitForService("/signalling/ear_leds_set_angle");
  ear_leds_angle_ = nh_->serviceClient<signalling_msgs::EarLedsSetAngle>(
                      "/signalling/ear_leds_set_angle", true);
  ros::service::waitForService("/signalling/fade");
  fade_led_ = nh_->serviceClient<signalling_msgs::Fade>(
                "/signalling/fade", true);
  ros::service::waitForService("/signalling/fade_rgb");
  fade_rgb_led_ = nh_->serviceClient<signalling_msgs::FadeRGB>(
                    "/signalling/fade_rgb", true);
  ros::service::waitForService("/signalling/rotate_eyes");
  rotate_eyes_ = nh_->serviceClient<signalling_msgs::RotateEyes>(
                   "/signalling/rotate_eyes", true);
  ros::service::waitForService("/signalling/set_intensity");
  set_intensity_led_ = nh_->serviceClient<signalling_msgs::SetIntensity>(
                         "/signalling/set_intensity", true);
  ros::service::waitForService("/signalling/say_text");
  speak_text_ = nh_->serviceClient<signalling_msgs::SayText>(
                  "/signalling/say_text", true);
  this->init();
}

Monitor::~Monitor() {
}

void Monitor::init() {
  right_eye_leds_.request.name = "RightFaceLeds";
  right_eye_leds_.request.duration = 0.0f;
  left_eye_top_.request.name = "FaceLedsLeftTop";
  left_eye_bottom_.request.name = "FaceLedsLeftBottom";
  right_eye_top_.request.name = "FaceLedsRightTop";
  right_eye_top_.request.duration = 0.0f;
  right_eye_bottom_.request.name = "FaceLedsRightBottom";
  right_eye_bottom_.request.duration = 0.0f;
  face_leds_.request.name = "FaceLeds";  // Left Face controlled from Vision
}

// Left Eye: Camera selection. Right Eye: State monitoring
bool Monitor::setMonitorMode(motion_planning_msgs::MonitorMode::Request& req,
                             motion_planning_msgs::MonitorMode::Response& res) {
  // ROS_INFO("YO");
  // Turn off all Face LEDs
  if (req.monitor_mode == MonitorMode::DISABLE) {
    face_leds_.request.intensity = 0.0f;
    set_intensity_led_.call(face_leds_);
    // Turn on bottom part of left eye LEDs
  } else if (req.monitor_mode == MonitorMode::BOTTOM_CAMERA) {
    left_eye_top_.request.intensity = 0.0f;
    set_intensity_led_.call(left_eye_top_);
    left_eye_bottom_.request.intensity = 1.0f;
    set_intensity_led_.call(left_eye_bottom_);
    // Turn on top part of left eye LEDs
  } else if (req.monitor_mode == MonitorMode::TOP_CAMERA) {
    left_eye_bottom_.request.intensity = 0.0f;
    set_intensity_led_.call(left_eye_bottom_);
    left_eye_top_.request.intensity = 1.0f;
    set_intensity_led_.call(left_eye_top_);
    // Turn on red LEDs on right eye
  } else if (req.monitor_mode == MonitorMode::BALL_SEEN) {
    right_eye_top_.request.rgb = Colors::RED;
    fade_rgb_led_.call(right_eye_top_);
    // Turn on blue LEDs on right eye
  } else if (req.monitor_mode == MonitorMode::PTAM_ACTIVE) {
    right_eye_bottom_.request.rgb = Colors::BLUE;
    fade_rgb_led_.call(right_eye_bottom_);
    // Turn off LEDs on right eye top
  } else if (req.monitor_mode == MonitorMode::BALL_LOST) {
    right_eye_top_.request.rgb = Colors::BLACK;
    fade_rgb_led_.call(right_eye_top_);
    // Turn off LEDs on right eye bottom
  } else if (req.monitor_mode == MonitorMode::PTAM_LOST) {
    right_eye_bottom_.request.rgb = Colors::BLACK;
    fade_rgb_led_.call(right_eye_bottom_);
  }

  return true;
}

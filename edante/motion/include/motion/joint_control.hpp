/**
 * @file      joint_control.h
 * @brief     Defines the Joint Control Wrapper functions
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef JOINT_CONTROL_HPP
#define JOINT_CONTROL_HPP

// NaoQi
#include <alerror/alerror.h>
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <althread/almutex.h>

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// Messages
#include <motion_msgs/Float32List.h>
#include <motion_msgs/AngleInterp.h>
#include <motion_msgs/AngleInterpSpeed.h>
#include <motion_msgs/SetAngles.h>
#include <motion_msgs/ChangeAngles.h>
#include <motion_msgs/GetAngles.h>
#include <motion_msgs/UseHand.h>


class JointControl : public AL::ALModule  {
 public:
  JointControl(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
  ~JointControl();

  void init();

  void rosSetup(ros::NodeHandle* nh);

  // ROS services
  bool angleInterp(motion_msgs::AngleInterp::Request& req,
                   motion_msgs::AngleInterp::Response& res);
  bool angleInterpSpeed(motion_msgs::AngleInterpSpeed::Request& req,
                        motion_msgs::AngleInterpSpeed::Response& res);
  bool setAngles(motion_msgs::SetAngles::Request& req,
                 motion_msgs::SetAngles::Response& res);
  bool changeAngles(motion_msgs::ChangeAngles::Request& req,
                    motion_msgs::ChangeAngles::Response& res);
  bool getAngles(motion_msgs::GetAngles::Request& req,
                 motion_msgs::GetAngles::Response& res);
  bool closeHand(motion_msgs::UseHand::Request& req,
                 motion_msgs::UseHand::Response& res);
  bool openHand(motion_msgs::UseHand::Request& req,
                motion_msgs::UseHand::Response& res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_angle_interp_;
  ros::ServiceServer srv_angle_interp_speed_;
  ros::ServiceServer srv_set_angles_;
  ros::ServiceServer srv_change_angles_;
  ros::ServiceServer srv_get_angles_;
  ros::ServiceServer srv_close_hand_;
  ros::ServiceServer srv_open_hand_;

  // NAOqi
  boost::shared_ptr<AL::ALMutex> fCallbackMutex;
  AL::ALMemoryProxy fMemoryProxy;
  AL::ALMotionProxy mProxy_;
};
#endif /* JOINT_CONTROL_HPP */

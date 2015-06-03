/**
 * @file      cartesian_control.h
 * @brief     ROS wrapper for NaoQI cartesian control methods
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef CARTESIAN_CONTROL_HPP
#define CARTESIAN_CONTROL_HPP

// System
#include <vector>

// NaoQi
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <althread/almutex.h>

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// Messages
#include <std_msgs/Bool.h>
#include <motion_msgs/PositionInterpolation.h>
#include <motion_msgs/PositionInterpolations.h>
#include <motion_msgs/SetPosition.h>
#include <motion_msgs/ChangePosition.h>
#include <motion_msgs/GetPosition.h>
#include <motion_msgs/GetTransform.h>

class CartesianControl : public AL::ALModule  {
 public:
  CartesianControl(boost::shared_ptr<AL::ALBroker> broker,
                   const std::string& name);
  ~CartesianControl();

  void init();

  void rosSetup(ros::NodeHandle* nh);

  // ROS services
  bool positionInterpolation(motion_msgs::PositionInterpolation::Request& req,
                             motion_msgs::PositionInterpolation::Response& res);
  bool positionInterpolations(
    motion_msgs::PositionInterpolations::Request& req,
    motion_msgs::PositionInterpolations::Response& res);
  bool setPosition(motion_msgs::SetPosition::Request& req,
                   motion_msgs::SetPosition::Response& res);
  bool changePosition(motion_msgs::ChangePosition::Request& req,
                      motion_msgs::ChangePosition::Response& res);
  bool getPosition(motion_msgs::GetPosition::Request& req,
                   motion_msgs::GetPosition::Response& res);
  bool getTransform(motion_msgs::GetTransform::Request& req,
                    motion_msgs::GetTransform::Response& res);


 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_position_interpolation_;
  ros::ServiceServer srv_position_interpolations_;
  ros::ServiceServer srv_set_position_;
  ros::ServiceServer srv_change_position_;
  ros::ServiceServer srv_get_position_;
  ros::ServiceServer srv_transform_interpolation_;
  ros::ServiceServer srv_transform_interpolations_;
  ros::ServiceServer srv_set_transform_;
  ros::ServiceServer srv_change_transform_;
  ros::ServiceServer srv_get_transform_;

  // NAOqi
  boost::shared_ptr<AL::ALMutex> fCallbackMutex;
  AL::ALMemoryProxy fMemoryProxy;
  AL::ALMotionProxy mProxy_;
};

#endif /* CARTESIAN_CONTROL_HPP */

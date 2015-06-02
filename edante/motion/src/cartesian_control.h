/**
 * @file      cartesian_control.h
 * @brief     ROS wrapper for NaoQI cartesian control methods
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef CARTESIAN_CONTROL_H_
#define CARTESIAN_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <althread/almutex.h>

#include <vector>
#include "motion/PositionInterpolation.h"
#include "motion/PositionInterpolations.h"
#include "motion/SetPosition.h"
#include "motion/ChangePosition.h"
#include "motion/GetPosition.h"
#include "motion/GetTransform.h"
#include "definitions.h"

class CartesianControl : public AL::ALModule  {
 public:
  CartesianControl(boost::shared_ptr<AL::ALBroker> broker,
                   const std::string& name);
  ~CartesianControl();

  void init();

  void rosSetup(ros::NodeHandle* nh);

  // ROS services
  bool positionInterpolation(motion::PositionInterpolation::Request &req,
                             motion::PositionInterpolation::Response &res);
  bool positionInterpolations(motion::PositionInterpolations::Request &req,
                              motion::PositionInterpolations::Response &res);
  bool setPosition(motion::SetPosition::Request &req,
                   motion::SetPosition::Response &res);
  bool changePosition(motion::ChangePosition::Request &req,
                      motion::ChangePosition::Response &res);
  bool getPosition(motion::GetPosition::Request &req,
                   motion::GetPosition::Response &res);
  bool getTransform(motion::GetTransform::Request &req,
                    motion::GetTransform::Response &res);


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

#endif /* CARTESIAN_CONTROL_H_ */

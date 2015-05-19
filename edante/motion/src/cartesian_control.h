/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-18
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI cartesian control methods
*/

#ifndef CARTESIAN_CONTROL_H_
#define CARTESIAN_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <alproxies/almotionproxy.h>

#include <vector>
#include "motion/PositionInterpolation.h"
#include "motion/PositionInterpolations.h"
#include "motion/SetPosition.h"
#include "motion/ChangePosition.h"
#include "motion/GetPosition.h"
#include "motion/GetTransform.h"
#include "definitions.h"

class CartesianControl {
 public:
  CartesianControl(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy);
  ~CartesianControl();

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
  AL::ALMotionProxy* mProxy_;
};

#endif /* CARTESIAN_CONTROL_H_ */

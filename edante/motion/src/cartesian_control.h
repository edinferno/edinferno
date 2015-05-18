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
#include "motion/positionInterpolation.h"
#include "motion/positionInterpolations.h"
#include "motion/setPosition.h"
#include "motion/changePosition.h"
#include "motion/getPosition.h"
#include "motion/getTransform.h"
#include "definitions.h"

class Cartesian_Control {
 public:
  Cartesian_Control(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy);
  ~Cartesian_Control();

  // ROS services
  bool positionInterpolation(motion::positionInterpolation::Request &req,
                             motion::positionInterpolation::Response &res);
  bool positionInterpolations(motion::positionInterpolations::Request &req,
                              motion::positionInterpolations::Response &res);
  bool setPosition(motion::setPosition::Request &req,
                   motion::setPosition::Response &res);
  bool changePosition(motion::changePosition::Request &req,
                      motion::changePosition::Response &res);
  bool getPosition(motion::getPosition::Request &req,
                   motion::getPosition::Response &res);
  bool getTransform(motion::getTransform::Request &req,
                    motion::getTransform::Response &res);


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

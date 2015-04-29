#ifndef CARTESIAN_CONTROL_H_
#define CARTESIAN_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <vector>

#include "motion/getPosition.h"
#include "motion/getTransform.h"

#include <alproxies/almotionproxy.h>

#include "definitions.h"

using namespace std;

class Cartesian_Control{
public:
 Cartesian_Control();
 ~Cartesian_Control();

// Cartesian control API
 vector<float> getPosition(string& name, int& space, bool& useSensorValues);
 vector<float> getTransform(string& name, int& space, bool& useSensorValues);

 // ROS services
 bool getPositionSrv(motion::getPosition::Request &req,
  motion::getPosition::Response &res);
 bool getTransformSrv(motion::getTransform::Request &req,
  motion::getTransform::Response &res);


private:
 // ROS
 ros::NodeHandle* nh_;
 ros::ServiceServer get_position_;
 ros::ServiceServer get_transform_;

 // NAOqi
 AL::ALMotionProxy* mProxy_;
};

#endif /* CARTESIAN_CONTROL_H_ */
/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Joystick remote control node
*/

#ifndef JOY_REMOTE_H
#define JOY_REMOTE_H

#include <ros/ros.h>
#include <cmath>

#include <sensor_msgs/Joy.h>

#include "motion/moveToward.h"

#include "definitions.h"

class joyRemote {
 public:
  joyRemote(ros::NodeHandle* nh);

  ~joyRemote();

  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  void sendCmd();

 private:
  // ROS
  motion::moveToward moveSrv;
  ros::ServiceClient moveClient;
  ros::NodeHandle* nh_;
  ros::Subscriber joy_sub_;

};

#endif  /* JOY_REMOTE_H */

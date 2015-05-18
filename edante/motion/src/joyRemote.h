/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Joystick remote control node
*/

#ifndef JOY_REMOTE_H
#define JOY_REMOTE_H

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <cmath>

#include "motion/moveToward.h"
#include "motion/setPosture.h"
#include "definitions.h"

class joyRemote {
 public:
  explicit joyRemote(ros::NodeHandle* nh);

  ~joyRemote();

  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  void sendCmd();

  void wakeUp();

  void moveInit();

  void stand();

  void rest();

 private:
  // Flags
  bool wakeUpFlag;
  bool moveInitFlag;
  bool standFlag;
  bool restFlag;

  // ROS
  ros::NodeHandle* nh_;
  ros::Subscriber joy_sub_;
  motion::moveToward moveSrv;
  std_srvs::Empty wakeUpSrv;
  std_srvs::Empty moveInitSrv;
  motion::setPosture standSrv;
  std_srvs::Empty restSrv;
  std_srvs::Empty stopSrv;
  ros::ServiceClient moveTowardClient;
  ros::ServiceClient wakeUpClient;
  ros::ServiceClient moveInitClient;
  ros::ServiceClient standClient;
  ros::ServiceClient restClient;
  ros::ServiceClient stopMoveClient;
};

#endif  /* JOY_REMOTE_H */

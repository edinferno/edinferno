/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's touch sensors
*/

#ifndef TOUCH_H
#define TOUCH_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "sensing/bumpers.h"
#include "sensing/head.h"
#include "sensing/hand.h"

#include <alproxies/almemoryproxy.h>

#include "definitions.h"

using namespace std;

class Touch {
 public:
  Touch(ros::NodeHandle* nh, AL::ALMemoryProxy* memProxy);
  ~Touch();

  // ROS publishers
  void spinTopics();

  void checkBumpers();
  void checkChest();
  void checkHead();
  void checkHands();
  void checkRightHand();
  void checkLeftHand();

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher bumpers_pub_;
  ros::Publisher chest_pub_;
  ros::Publisher head_pub_;
  ros::Publisher right_hand_pub_;
  ros::Publisher left_hand_pub_;

  // NaoQI
  AL::ALMemoryProxy* memProxy_;
};

#endif  /* TOUCH_H */

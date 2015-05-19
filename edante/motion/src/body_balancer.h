/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-10
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI Whole Body Balancer
*/

#ifndef BODY_BALANCER_H_
#define BODY_BALANCER_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <alproxies/almotionproxy.h>

#include "motion/Enable.h"
#include "motion/FootState.h"
#include "motion/EnableBalanceConstraint.h"
#include "motion/GoToBalance.h"
#include "motion/EnableEffector.h"
#include "motion/SetEffectorControl.h"

#include "definitions.h"

class BodyBalancer {
 public:
  BodyBalancer(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy);
  ~BodyBalancer();

  // ROS publisher
  void spinTopics();

  // ROS services
  bool enableBalance(motion::Enable::Request &req,
                     motion::Enable::Response &res);
  bool footState(motion::FootState::Request &req,
                 motion::FootState::Response &res);
  bool enableBalanceConstraint(motion::EnableBalanceConstraint::Request &req,
                               motion::EnableBalanceConstraint::Response &res);
  bool goToBalance(motion::GoToBalance::Request &req,
                   motion::GoToBalance::Response &res);
  bool enableEffectorControl(motion::EnableEffector::Request &req,
                             motion::EnableEffector::Response &res);
  bool setEffectorControl(motion::SetEffectorControl::Request &req,
                          motion::SetEffectorControl::Response &res);
  bool enableEffectorOptimization(motion::EnableEffector::Request &req,
                                  motion::EnableEffector::Response &res);

 private:
// Flags
  bool isBalancing_;

// ROS
  ros::NodeHandle* nh_;
  ros::Publisher balance_pub_;
  ros::ServiceServer srv_enable_balance_;
  ros::ServiceServer srv_foot_state_;
  ros::ServiceServer srv_enable_balance_constraint_;
  ros::ServiceServer srv_go_to_balance_;
  ros::ServiceServer srv_enable_effector_control_;
  ros::ServiceServer srv_set_effector_control_;
  ros::ServiceServer srv_enable_effector_optimization_;

// NaoQI
  AL::ALMotionProxy* mProxy_;
};

#endif /* BODY_BALANCER_H_ */

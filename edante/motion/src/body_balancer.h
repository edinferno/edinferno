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

#include "motion/enable.h"
#include "motion/footState.h"
#include "motion/enableBalanceConstraint.h"
#include "motion/goToBalance.h"
#include "motion/enableEffector.h"
#include "motion/setEffectorControl.h"

#include "definitions.h"

class Body_Balancer {
 public:
  Body_Balancer(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy);
  ~Body_Balancer();

  // ROS publisher
  void spinTopics();

  // ROS services
  bool enableBalance(motion::enable::Request &req,
                     motion::enable::Response &res);
  bool footState(motion::footState::Request &req,
                 motion::footState::Response &res);
  bool enableBalanceConstraint(motion::enableBalanceConstraint::Request &req,
                               motion::enableBalanceConstraint::Response &res);
  bool goToBalance(motion::goToBalance::Request &req,
                   motion::goToBalance::Response &res);
  bool enableEffectorControl(motion::enableEffector::Request &req,
                             motion::enableEffector::Response &res);
  bool setEffectorControl(motion::setEffectorControl::Request &req,
                          motion::setEffectorControl::Response &res);
  bool enableEffectorOptimization(motion::enableEffector::Request &req,
                                  motion::enableEffector::Response &res);

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

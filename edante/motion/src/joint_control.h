/**
 * @file      joint_control.h
 * @brief     Defines the Joint Control Wrapper functions
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef JOINT_CONTROL_H_
#define JOINT_CONTROL_H_

#include <ros/ros.h>

#include <alerror/alerror.h>
#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <althread/almutex.h>

#include "motion/Float32List.h"
#include "motion/AngleInterp.h"
#include "motion/AngleInterpSpeed.h"
#include "motion/SetAngles.h"
#include "motion/ChangeAngles.h"
#include "motion/GetAngles.h"
#include "motion/UseHand.h"
#include "definitions.h"

class JointControl : public AL::ALModule  {
 public:
  JointControl(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
  ~JointControl();

  void init();

  void rosSetup(ros::NodeHandle* nh);

  // ROS services
  bool angleInterp(motion::AngleInterp::Request &req,
                   motion::AngleInterp::Response &res);
  bool angleInterpSpeed(motion::AngleInterpSpeed::Request &req,
                        motion::AngleInterpSpeed::Response &res);
  bool setAngles(motion::SetAngles::Request &req,
                 motion::SetAngles::Response &res);
  bool changeAngles(motion::ChangeAngles::Request &req,
                    motion::ChangeAngles::Response &res);
  bool getAngles(motion::GetAngles::Request &req,
                 motion::GetAngles::Response &res);
  bool closeHand(motion::UseHand::Request &req,
                 motion::UseHand::Response &res);
  bool openHand(motion::UseHand::Request &req,
                motion::UseHand::Response &res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_angle_interp_;
  ros::ServiceServer srv_angle_interp_speed_;
  ros::ServiceServer srv_set_angles_;
  ros::ServiceServer srv_change_angles_;
  ros::ServiceServer srv_get_angles_;
  ros::ServiceServer srv_close_hand_;
  ros::ServiceServer srv_open_hand_;

  // NAOqi
  boost::shared_ptr<AL::ALMutex> fCallbackMutex;
  AL::ALMemoryProxy fMemoryProxy;
  AL::ALMotionProxy mProxy_;
};
#endif /* JOINT_CONTROL_H_ */

/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-18
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Defines the Joint Control Wrapper functions
*/

#ifndef JOINT_CONTROL_H_
#define JOINT_CONTROL_H_

#include <ros/ros.h>

#include <alproxies/almotionproxy.h>
#include <alerror/alerror.h>

#include "motion/float32List.h"
#include "motion/angleInterp.h"
#include "motion/angleInterpSpeed.h"
#include "motion/setAngles.h"
#include "motion/changeAngles.h"
#include "motion/getAngles.h"
#include "motion/useHand.h"
#include "definitions.h"

class Joint_Control {
 public:
  Joint_Control(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy);
  ~Joint_Control();

  // ROS services
  bool angleInterp(motion::angleInterp::Request &req,
                   motion::angleInterp::Response &res);
  bool angleInterpSpeed(motion::angleInterpSpeed::Request &req,
                        motion::angleInterpSpeed::Response &res);
  bool setAngles(motion::setAngles::Request &req,
                 motion::setAngles::Response &res);
  bool changeAngles(motion::changeAngles::Request &req,
                    motion::changeAngles::Response &res);
  bool getAngles(motion::getAngles::Request &req,
                 motion::getAngles::Response &res);
  bool closeHand(motion::useHand::Request &req,
                 motion::useHand::Response &res);
  bool openHand(motion::useHand::Request &req,
                motion::useHand::Response &res);

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
  AL::ALMotionProxy* mProxy_;
};
#endif /* JOINT_CONTROL_H_ */

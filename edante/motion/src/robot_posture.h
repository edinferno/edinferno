/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-10
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI ALRobotPosture
*/
#ifndef ROBOT_POSTURE_H_
#define ROBOT_POSTURE_H_

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <alproxies/alrobotpostureproxy.h>

#include "motion/getPostureList.h"
#include "motion/setPosture.h"
#include "motion/getPostureFamily.h"
#include "motion/setMaxTryNumber.h"

#include "definitions.h"

using namespace std;

class Robot_Posture {
 public:
  Robot_Posture(ros::NodeHandle* nh, AL::ALRobotPostureProxy* pProxy);
  ~Robot_Posture();

  // ROS services
  bool getPostureList(motion::getPostureList::Request &req,
                      motion::getPostureList::Response &res);
  bool goToPosture(motion::setPosture::Request &req,
                   motion::setPosture::Response &res);
  bool applyPosture(motion::setPosture::Request &req,
                    motion::setPosture::Response &res);
  bool stopPosture(std_srvs::Empty::Request &req,
                   std_srvs::Empty::Response &res);
  bool getPostureFamily(motion::getPostureFamily::Request &req,
                        motion::getPostureFamily::Response &res);
  bool getPostureFamilyList(motion::getPostureList::Request &req,
                            motion::getPostureList::Response &res);
  bool setMaxTryNumber(motion::setMaxTryNumber::Request &req,
                       motion::setMaxTryNumber::Response &res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_get_posture_list_;
  ros::ServiceServer srv_go_to_posture_;
  ros::ServiceServer srv_apply_posture_;
  ros::ServiceServer srv_stop_posture_;
  ros::ServiceServer srv_get_posture_family_;
  ros::ServiceServer srv_get_posture_family_list_;
  ros::ServiceServer srv_set_max_try_number_;

  // NaoQI
  AL::ALRobotPostureProxy* pProxy_;

};

#endif /* ROBOT_POSTURE_H_ */

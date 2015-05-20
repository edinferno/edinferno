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

#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <althread/almutex.h>

#include "motion/GetPostureList.h"
#include "motion/SetPosture.h"
#include "motion/GetPostureFamily.h"
#include "motion/SetMaxTryNumber.h"
#include "definitions.h"

class RobotPosture : public AL::ALModule  {
 public:
  RobotPosture(boost::shared_ptr<AL::ALBroker> broker,
               const std::string& name);
  ~RobotPosture();

  void init();

  void rosSetup(ros::NodeHandle* nh);

  // ROS services
  bool getPostureList(motion::GetPostureList::Request &req,
                      motion::GetPostureList::Response &res);
  bool goToPosture(motion::SetPosture::Request &req,
                   motion::SetPosture::Response &res);
  bool applyPosture(motion::SetPosture::Request &req,
                    motion::SetPosture::Response &res);
  bool stopPosture(std_srvs::Empty::Request &req,
                   std_srvs::Empty::Response &res);
  bool getPostureFamily(motion::GetPostureFamily::Request &req,
                        motion::GetPostureFamily::Response &res);
  bool getPostureFamilyList(motion::GetPostureList::Request &req,
                            motion::GetPostureList::Response &res);
  bool setMaxTryNumber(motion::SetMaxTryNumber::Request &req,
                       motion::SetMaxTryNumber::Response &res);

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
  boost::shared_ptr<AL::ALMutex> fCallbackMutex;
  AL::ALMemoryProxy fMemoryProxy;
  AL::ALRobotPostureProxy pProxy_;
};

#endif /* ROBOT_POSTURE_H_ */

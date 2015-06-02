/**
 * @file      stiffness_control.h
 * @brief     ROS wrapper for NaoQI Stiffness control API
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef STIFFNESS_CONTROL_H_
#define STIFFNESS_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <alerror/alerror.h>

#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <althread/almutex.h>

#include <vector>
#include "motion/StiffnessInterp.h"
#include "motion/SetStiffness.h"
#include "motion/GetStiffness.h"
#include "definitions.h"

using std::string;
using std::vector;

class StiffnessControl : public AL::ALModule  {
 public:
  StiffnessControl(boost::shared_ptr<AL::ALBroker> broker,
                   const std::string& name);
  ~StiffnessControl();

  void init();

  void rosSetup(ros::NodeHandle* nh);

// ROS services
  bool wakeUp(std_srvs::Empty::Request &req,
              std_srvs::Empty::Response &res);
  bool rest(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &res);
  bool stiffnessInterp(motion::StiffnessInterp::Request &req,
                       motion::StiffnessInterp::Response &res);
  bool setStiffness(motion::SetStiffness::Request &req,
                    motion::SetStiffness::Response &res);
  bool getStiffness(motion::GetStiffness::Request &req,
                    motion::GetStiffness::Response &res);

  // Stiffness control API
  bool setStiffnesses(const string& name, const float& stiffness);
  bool setStiffnesses(const string& name, const vector<float>& stiffnesses);
  bool setStiffnesses(const vector<string>& names, const float& stiffness);
  bool setStiffnesses(const vector<string>& names,
                      const vector<float>& stiffnesses);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::Publisher wake_pub_;
  ros::ServiceServer stiffness_interp_;
  ros::ServiceServer set_stiffness_;
  ros::ServiceServer get_stiffness_;
  ros::ServiceServer srv_wake_up_;
  ros::ServiceServer srv_rest_;

  std_msgs::Bool awake_;

  // NAOqi
  boost::shared_ptr<AL::ALMutex> fCallbackMutex;
  AL::ALMotionProxy mProxy_;
  AL::ALMemoryProxy fMemoryProxy;
};

#endif /* STIFFNESS_CONTROL_H_ */

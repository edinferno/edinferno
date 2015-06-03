/**
 * @file      stiffness_control.h
 * @brief     ROS wrapper for NaoQI Stiffness control API
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-05-20
 * @copyright (MIT) 2015 Edinferno
 */

#ifndef STIFFNESS_CONTROL_HPP
#define STIFFNESS_CONTROL_HPP

// System
#include <vector>

// NaoQi
#include <alerror/alerror.h>
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <althread/almutex.h>

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>
// Messages
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <motion_msgs/StiffnessInterp.h>
#include <motion_msgs/SetStiffness.h>
#include <motion_msgs/GetStiffness.h>

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
  bool wakeUp(std_srvs::Empty::Request& req,
              std_srvs::Empty::Response& res);
  bool rest(std_srvs::Empty::Request& req,
            std_srvs::Empty::Response& res);
  bool stiffnessInterp(motion_msgs::StiffnessInterp::Request& req,
                       motion_msgs::StiffnessInterp::Response& res);
  bool setStiffness(motion_msgs::SetStiffness::Request& req,
                    motion_msgs::SetStiffness::Response& res);
  bool getStiffness(motion_msgs::GetStiffness::Request& req,
                    motion_msgs::GetStiffness::Response& res);

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

#endif /* STIFFNESS_CONTROL_HPP */

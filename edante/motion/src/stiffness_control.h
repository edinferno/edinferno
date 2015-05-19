/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-18
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI Stiffness control API
*/

#ifndef STIFFNESS_CONTROL_H_
#define STIFFNESS_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <alproxies/almotionproxy.h>
#include <alerror/alerror.h>

#include <vector>
#include "motion/StiffnessInterp.h"
#include "motion/SetStiffness.h"
#include "motion/GetStiffness.h"
#include "definitions.h"

using std::string;
using std::vector;

class StiffnessControl {
 public:
  StiffnessControl(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy);
  ~StiffnessControl();

// ROS publisher
  void spinTopics();

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

// NAOqi
  AL::ALMotionProxy* mProxy_;

// Internal
  bool awake_;
};

#endif /* STIFFNESS_CONTROL_H_ */

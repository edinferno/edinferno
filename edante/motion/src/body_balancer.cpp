/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-10
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for NaoQI Whole Body Balancer
*/

#include "body_balancer.h"

Body_Balancer::Body_Balancer(ros::NodeHandle* nh, AL::ALMotionProxy* mProxy) {
  nh_ = nh;
  mProxy_ = mProxy;
  INFO("Setting up Body Balancer services" << std::endl);
  // srv_change_transform_ = nh_->advertiseService("changeTransform",
  //                         &Body_Balancer::changeTransform, this);
}

Body_Balancer::~Body_Balancer() {
  ros::shutdown();
}

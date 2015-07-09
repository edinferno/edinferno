/**
 * @file      localization.cpp
 * @brief     Main class for Nao localization
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-07-07
 * @copyright (MIT) 2015 Edinferno
 */

#include "localization/ptam_wrapper.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization");
  ros::NodeHandle nh("localization");
  ROS_INFO("Running Localization");
  PTAMWrapper ptam_wrapper(nh);

  ros::spin();

  ros::shutdown();

  return 0;
}

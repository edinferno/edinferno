/**
 * @file      motion_planning.cpp
 * @brief     Main motion planning node
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion_planning/stand_up.hpp"
#include "motion_planning/sit_down.hpp"
#include "motion_planning/sit_rest.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_planning");
  ros::NodeHandle nh("motion_planning");
  ROS_INFO("Running Motion Planning");
  StandUpAction stand(nh, "stand_up");
  SitDownAction sit(nh, "sit_down");
  SitRestAction rest(nh, "sit_rest");
  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  ros::shutdown();

  return 0;
}

/**
 * @file      motion_planning.cpp
 * @brief     Main motion planning node
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */

#include "motion_planning/setup.hpp"
#include "motion_planning/stand_up.hpp"
#include "motion_planning/sit_down.hpp"
#include "motion_planning/sit_rest.hpp"
#include "motion_planning/await_transition.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_planning");
  ros::NodeHandle nh("motion_planning");
  ROS_INFO("Running Motion Planning");
  SetupAction setup(nh, "setup");
  StandUpAction stand(nh, "stand_up");
  SitDownAction sit(nh, "sit_down");
  SitRestAction rest(nh, "sit_rest");
  AwaitTransitionAction await(nh, "await_transition");

  ros::spin();

  ros::shutdown();

  return 0;
}

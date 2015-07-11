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
#include "motion_planning/kick_strong.hpp"
#include "motion_planning/transition.hpp"
#include "motion_planning/monitor.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_planning");
  ros::NodeHandle nh("motion_planning");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ROS_INFO("Running Motion Planning");
  Monitor monitor(&nh);
  SetupAction setup(nh, "setup");
  StandUpAction stand(nh, "stand_up");
  SitDownAction sit(nh, "sit_down");
  SitRestAction rest(nh, "sit_rest");
  KickStrongAction kick(nh, "kick_strong");
  TransitionAction await(nh, "transition");

  ros::waitForShutdown();
  ros::shutdown();

  return 0;
}

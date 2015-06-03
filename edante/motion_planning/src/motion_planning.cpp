/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-22
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Main motion planning node
*/

#include "motion_planning/stand_server.hpp"
#include "motion_planning/sit_server.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_planning");
  ROS_INFO("Running Motion Planning");
  StandAction stand("stand_up_action");
  SitAction sit("sit_down_action");
  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  ros::shutdown();

  return 0;
}

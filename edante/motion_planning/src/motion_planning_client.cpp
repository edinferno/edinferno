/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-22
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Test client for calling different actions, to be put in Agent
*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_planning_msgs/StandAction.h>
#include <motion_planning_msgs/SitAction.h>

int main (int argc, char** argv) {
  ros::init(argc, argv, "stand_client");

  // CHOOSE ACTION TO START
  // std::string ac_name = "stand_up_action";
  std::string ac_name = "sit_down_action";

  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<motion_planning_msgs::StandAction> ac(ac_name,
                                                                      true);

  ROS_INFO("Waiting for %s to start.", ac_name.c_str());
  ac.waitForServer();

  ROS_INFO("%s started, sending goal.", ac_name.c_str());
  motion_planning_msgs::StandGoal goal;
  goal.goal = true;
  ac.sendGoal(goal);

  bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  } else
  { ROS_INFO("Action did not finish before the time out."); }

  ros::shutdown();

  return 0;
}

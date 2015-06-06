#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <navigation_msgs/WalkToBallAction.h>

int main (int argc, char** argv) {
  ros::init(argc, argv, "nav_client");

  std::string ac_name = "navigation";
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<navigation_msgs::WalkToBallAction>
  ac(ac_name, true);

  ROS_INFO("Waiting for %s to start.", ac_name.c_str());
  ac.waitForServer();

  ROS_INFO("%s started, sending goal.", ac_name.c_str());
  navigation_msgs::WalkToBallGoal goal;
  goal.start_walk = true;
  ac.sendGoal(goal);

  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  } else
  { ROS_INFO("Action did not finish before the time out."); }

  ros::shutdown();

  return 0;
}
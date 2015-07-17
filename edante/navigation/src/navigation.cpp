#include "navigation/move_to.hpp"
#include "navigation/turn_to_pose.hpp"
#include "navigation/walk_to_pose.hpp"
#include "navigation/align_to_pose.hpp"
#include "navigation/walk_to_ball.hpp"
#include "navigation/search_for_ball.hpp"
#include "navigation/look_at_pose.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation");
  ros::NodeHandle nh("navigation");
  // ros::AsyncSpinner spinner(3);
  // spinner.start();
  ROS_INFO("Running Navigation");
  MoveToAction move_to(nh, "move_to");
  TurnToPoseAction turn_to_pose(nh, "turn_to_pose");
  WalkToPoseAction walk_to_pose(nh, "walk_to_pose");
  AlignToPoseAction align_to_pose(nh, "align_to_pose");
  WalkToBallAction walk_to_ball(nh, "walk_to_ball");
  SearchForBallAction search_for_ball(nh, "search_for_ball");
  LookAtPoseAction look_at_pose(nh, "look_at_pose");

  ros::spin();
  // ros::waitForShutdown();
  ros::shutdown();

  return 0;
}

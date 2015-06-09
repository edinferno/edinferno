#include "navigation/move_to.hpp"
#include "navigation/walk_to_ball.hpp"
#include "navigation/search_for_ball.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation");
  ros::NodeHandle nh("navigation");
  ROS_INFO("Running Navigation");
  MoveToAction move_to(nh, "move_to");
  WalkToBallAction walk_to_ball(nh, "walk_to_ball");
  SearchForBallAction search_for_ball(nh, "search_for_ball");
  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  ros::shutdown();

  return 0;
}

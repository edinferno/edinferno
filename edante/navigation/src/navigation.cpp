#include "navigation/navigation_server.hpp"
#include "navigation/walk_to_ball_server.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation");
  ros::NodeHandle nh("navigation");
  ROS_INFO("Running Navigation");
  // NavigateAction navigation(ros::this_node::getName());
  WalkToBallAction walk_to_ball(nh, "walk_to_ball");
  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  ros::shutdown();

  return 0;
}

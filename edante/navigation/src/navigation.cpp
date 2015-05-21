#include "navigation_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation");
  ROS_INFO("Running Navigation");
  NavigateAction navigation(ros::this_node::getName());
  ros::spin();

  return 0;
}

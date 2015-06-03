#include "navigation/navigation_server.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation");
  ROS_INFO("Running Navigation");
  NavigateAction navigation(ros::this_node::getName());
  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  ros::shutdown();

  return 0;
}

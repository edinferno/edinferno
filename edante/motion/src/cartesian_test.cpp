#include <ros/ros.h>
#include "cartesian_control.h"
#include "definitions.h"

int main(int argc, char *argv[]) {
  using namespace std;
  ros::init(argc, argv, "cartesian_control");
  ros::NodeHandle nh("motion");
  AL::ALMotionProxy mProxy("127.0.0.1", 9559);
  Cartesian_Control CartesianTest(&nh, &mProxy);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}

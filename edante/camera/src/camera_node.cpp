/*
* @Copyright: Copyright[2015]<Svetlin Penkov>
*      @Date: 2015-05-24
*     @Email: s.v.penkov@sms.ed.ac.uk
*      @Desc: This is the main program which starts both cameras of the NAO.
*/
#include <ros/ros.h>

#include "camera/camera.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_node");
  ros::NodeHandle nh("camera");

  Camera c(nh, 0, "top");

  if (!c.Init()) {
    return 1;
  }

  ros::Rate rate(5);
  while (ros::ok()) {

    c.SpinOnce();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

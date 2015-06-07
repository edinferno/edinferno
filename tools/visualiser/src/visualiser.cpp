/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-06
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The visualiser node listens to information from the robot
*             and converts it to a format suitable for RViz.
*/
#include <ros/ros.h>

#include "visualiser/visualise_vision.hpp"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "visualiser");

  ros::NodeHandle nh("visualiser");
  ros::Rate rate(30);

  VisualiseVision vision(nh);

  while (ros::ok()) {
    vision.Spin();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

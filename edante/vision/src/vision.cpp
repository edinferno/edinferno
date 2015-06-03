/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-01
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: This is the main file of the vision module.
*/
#include <ros/ros.h>

#include "vision/vision_node.hpp"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "vision_node");

  VisionNode vision_node;
  vision_node.Spin();
  return 0;
}

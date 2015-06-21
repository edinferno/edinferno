/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-06-21
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: The comms creates and runs a CommsNode.
*/

#include <ros/ros.h>

#include "comms/comms_node.hpp"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "comms_node");

  CommsNode comms_node;
  comms_node.Spin();
  return 0;
}

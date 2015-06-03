/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-17
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Main signalling node
*/

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>

#include "signalling/led.hpp"
#include "signalling/audio.hpp"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "signalling");
  ros::NodeHandle nh("signalling");

  Led LedTest(&nh);
  Audio AudioTest(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  ros::shutdown();
}

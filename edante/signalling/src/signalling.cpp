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

  std::string naoqi_ip_;
  int naoqi_port_;
  ros::param::param<std::string>("/naoqi_ip", naoqi_ip_, "127.0.0.1");
  ros::param::param("/naoqi_port", naoqi_port_, 9559);

  Led LedTest(&nh, naoqi_ip_, naoqi_port_);
  Audio AudioTest(&nh, naoqi_ip_, naoqi_port_);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  ros::shutdown();
}

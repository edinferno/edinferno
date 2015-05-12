/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-11
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS Wrapper for NaoQI sensor events
*/

#include <ros/ros.h>
#include "definitions.h"

#include "touch.h"
#include "power.h"
#include "sonar.h"
#include "fsr.h"

#include <alproxies/almemoryproxy.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "sensing");
  ros::NodeHandle nh("sensing");
  AL::ALMemoryProxy memProxy("127.0.0.1", 9559);
  Touch TouchTest(&nh, &memProxy);
  // Power PowerTest(&nh, &memProxy);
  // Sonar SonarTest(&nh, &memProxy);
  // Fsr FsrTest(&nh, &memProxy);

  ros::Rate r(10);

  while (ros::ok()) {
    TouchTest.spinTopics();
    // PowerTest.spinTopics();
    // SonarTest.spinTopics();
    // FsrTest.spinTopics();
    ros::spinOnce();
    r.sleep();
  }

}

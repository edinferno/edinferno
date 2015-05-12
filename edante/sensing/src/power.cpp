/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's power sensors
*/

#include "power.h"

Power::Power() {

}

Power::~Power() {
  ros::shutdown();
}

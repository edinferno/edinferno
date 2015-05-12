/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's fsr sensors
*/

#include "fsr.h"

Fsr::Fsr() {

}

Fsr::~Fsr() {
  ros::shutdown();
}

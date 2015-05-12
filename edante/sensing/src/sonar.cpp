/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's sonar sensors
*/

#include "sonar.h"

Sonar::Sonar() {

}

Sonar::~Sonar() {
  ros::shutdown();
}

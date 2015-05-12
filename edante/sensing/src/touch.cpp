/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-12
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      ROS wrapper for Nao's touch sensors
*/

#include "touch.h"

Touch::Touch() {

}

Touch::~Touch() {
  ros::shutdown();
}

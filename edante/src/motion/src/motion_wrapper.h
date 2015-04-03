//  motion_wrapper.h
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <alvalue/alvalue.h>
#include <alproxies/almotionproxy.h>

std_msgs::bool awake;

class Motion{

	//stiffness control
	void wakeUp();
	void rest();
	void setStiffnesses(const std::vector<std::string> names, const std::vector<float> stiffnesses);
	std::vector<float> getStiffnesses(const std::vector<std::string> names);


};



//  motion_wrapper.h

#include <vector>
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

    //joint control
    void angleInterpolation(const std::vector<std::string> names, const std::vector<float> angleLists, const std::vector<float> timeLists, const bool isAbsolute)
    void angleInterpolationWithSpeed(const std::vector<std::string> names, const std::vector<float> targetAngles, const float maxSpeedFraction)
    void angleInterpolationBezier(const std::vector<std::string> jointNames, const std::vector<float> times, const std::vector<std::vector<float angle, std::vector<int interpolationType, float dAngle, float dTime> Handle1, std::vector<int interpolationType, float dAngle, float dTime> Handle2>> controlPoints)
    void setAngles(const std::vector<std::string> names, const std::vector<float> angles, const float fractionMaxSpeed)
    void changeAngles(const std::vector<std::string> names, const std::vector<float> changes, const float fractionMaxSpeed)
    std::vector<float> getAngles(const std::vector<std::string> names, const bool useSensors)
    void closeHand(const std::string handName)
};


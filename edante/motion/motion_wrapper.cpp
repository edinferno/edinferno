//  motion_wrapper.cpp

#include "motion_wrapper.h"
#include "ros/ros.h"
#include <vector>
#include <alvalue/alvalue.h>
#include <alproxies/almotionproxy.h>

std_msgs::bool awake;

void wakeUp(AL::ALMotionproxy& motion){
    motion.wakeUp();
    msg.data = TRUE;
}

void rest(AL::ALMotionproxy& motion){
    motion.rest();
}

void setStiffness(AL::ALMotionproxy& motion, const std::vector<std::string>& names, const std::vector<double>& stiffnesses){
    motion.setStiffness(ALValue(names), ALValue(stiffnesses));
}

std::vector<float> getStiffnesses(AL::ALMotionproxy& motion, const std::vector<std::string>& names){
    return ToFloatArray(motion.getStiffnesses(ALValue(names)));
    
    
    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "isAwake");
    ros::NodeHandle n;
    ros::Publisher wake_pub = n.advertise<std_msgs::bool>("isAwake"), 1000);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        wake_pub.publish(awake);
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void angleInterpolation(AL::ALMotionproxy& motion, const std::vector<std::string>& names, const std::vector<float>& angleLists, const std::vector<float>& timeLists, const bool& isAbsolute){
    motion.angleInterpolation(ALValue(names), ALValue(angleLists), ALValue(timeLists), bool(isAbsolute));
}

void angleInterpolationWithSpeed(AL::ALMotionproxy& motion, const std::vector<std::string>& names, const std::vector<float>& targetAngles, const float& maxSpeedFraction){
    motion.angleInterpolationWithSpeed(ALValue(names), ALValue(targetAngles), float(maxSpeedFraction));
}

void angleInterpolationBezier(AL::ALMotionproxy& motion, const std::vector<std::string>& jointNames, const std::vector<float>& times, const std::vector<std::vector<float angle, std::vector<int interpolationType, float dAngle, float dTime> Handle1, std::vector<int interpolationType, float dAngle, float dTime> Handle2>>& controlPoints){
    motion.angleInterpolationBezier(std::vector<std::string>(jointNames), ALValue(times), ALValue(controlPoints));
}

void setAngles(AL::ALMotionproxy& motion, const std::vector<std::string>& names, const std::vector<float>& angles, const float& fractionMaxSpeed){
    motion.setAngles(ALValue(names), ALValue(angles), float(fractionMaxSpeed));
}

void changeAngles(AL::ALMotionproxy& motion, const std::vector<std::string>& names, const std::vector<float>& changes, const float& fractionMaxSpeed){
    motion.changeAngles(ALValue(names), ALValue(changes), float(fractionMaxSpeed));
}

std::vector<float> getAngles(AL::ALMotionproxy& motion, const std::vector<std::string>& names, const bool& useSensors){
    return motion.getAngles(ALValue(names), bool(useSensors));
}

void closeHand(AL::ALMotionproxy& motion, const std::string& handName){
    motion.closeHand(std::string(handName));
}


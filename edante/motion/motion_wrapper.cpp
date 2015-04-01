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

void setStiffness(AL::ALMotionproxy& motion, const std::vector<std::string> names, const std::vector<double>stiffnesses){
    motion.setStiffness(ALValue(names), ALValue(stiffnesses));
}

std::vector<float> getStiffnesses(AL::ALMotionproxy& motion, const std::vector<std::string> names){
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
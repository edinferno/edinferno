#include "ros/ros.h"
#include "std_msgs/String.h"

#include "Networking.h"
#include "SPLStandardMessage.h"

typedef std_msgs::String OurMsg;

OurMsg convert() {
    OurMsg msg_; msg_.data = "y";
    return msg_;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "net");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<OurMsg>("net/spl_std_msg", 1000);
    ros::Rate loopRate(5);
    while (ros::ok()) {
        pub.publish(convert());
        ros::spinOnce();
        loopRate.sleep();
    }
}
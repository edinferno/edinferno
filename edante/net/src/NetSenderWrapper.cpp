#include "ros/ros.h"
#include "std_msgs/String.h"

#include "NetSender.h"
#include "comms/SPLStdMsg.h"
#include "SPLStandardMessage.h"

typedef std_msgs::String OurMsg;


void netSendCallback(const comms::SPLStdMsg& msg) {
    NetSender sender;
    sender.sendStandardMessage(convertFromROS(msg));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "net_sender");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("comms/spl_std_msg", 1000, netSendCallback);
    ros::spin();
}
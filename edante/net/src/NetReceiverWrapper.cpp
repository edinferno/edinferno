#include "ros/ros.h"
#include "std_msgs/String.h"

#include "NetReceiver.h"
#include "SPLStandardMessage.h"

typedef std_msgs::String OurMsg;

OurMsg convert() {
    OurMsg msg_; msg_.data = "y";
    return msg_;
}

void chatterCallback(const OurMsg::ConstPtr& msg) {
    printf("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "net_receiver");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("net_out/spl_std_msg", 1000, chatterCallback);
    ros::spin();
}
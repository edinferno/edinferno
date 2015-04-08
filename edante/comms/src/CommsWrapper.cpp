#include "ros/ros.h"
#include "std_msgs/String.h"
#include "comms/SPLStandardMessage.h"

#include "Communication.h"
#include "SPLStandardMessage.h"

typedef comms::SPLStandardMessage OurMsg;

OurMsg convert(SPLStandardMessage msg) {
    OurMsg msg_; msg_.msg = "x";
    return msg_;
}

int main(int argc, char** argv) {
    Communication c;

    ros::init(argc, argv, "comms");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<OurMsg>("comms/spl_std_msg", 1000);
    ros::Rate loopRate(5);
    while (ros::ok()) {
        pub.publish(convert(c.getStandardMessage()));
        ros::spinOnce();
        loopRate.sleep();
    }
}

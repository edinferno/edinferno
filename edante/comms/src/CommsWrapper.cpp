#include "ros/ros.h"
#include "std_msgs/String.h"
#include "comms/SPLStandardMessage.h"

#include "Communication.h"
#include "SPLStandardMessage.h"

int main(int argc, char** argv) {
    Communication c;

    ros::init(argc, argv, "comms");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<comms::SPLStandardMessage>(
        "comms/spl_std_msg",
        1000
    );
    ros::Rate loopRate(5);
    while (ros::ok()) {
        pub.publish(convertToROS(c.getStandardMessage()));
        ros::spinOnce();
        loopRate.sleep();
    }
}

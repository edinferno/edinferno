#include "ros/ros.h"
#include "std_msgs/String.h"

#include "NetReceiver.h"
#include "SPLStandardMessage.h"
#include "comms/SPLStdMsg.h"

/****************************************
 * To run tests:
 * 
 * 1: go: to the catkin_ws
 * 2: do: catkin_make
 * 3: do: source devel/setup.bash
 * 4: do: roslaunch comms launcher.launch
*******************************************/

typedef std_msgs::String OurMsg;

OurMsg convert() {
    OurMsg msg_; msg_.data = "y";
    return msg_;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "net_receiver");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<comms::SPLStdMsg>("net_out/spl_std_msg", 1000);
    NetReceiver receiver;
    while (ros::ok()) {
        SPLStandardMessage splMsg = receiver.receiveStandardMessage();
        pub.publish(convertToROS(splMsg));
        ros::spinOnce();
    }
    ros::spin();
}

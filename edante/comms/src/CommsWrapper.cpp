#include "ros/ros.h"
#include "std_msgs/String.h"
#include "comms/SPLStandardMessage.h"

#include "Communication.h"
#include "SPLStandardMessage.h"

void fillDummyData(Communication& comms) {
    Timestamp dummyTimestamp{0};
    Pose2D dummyPose{0.3456f, 0.4f, 0.8f};
    Point2D dummyWalk{0.3f, 0.5f};
    Point2D dummyShoot{0.5f, 0.7f};
    Point2D dummyBallPos{0.6f, 0.8f};
    Vector2D dummyBallVel{0.7f, 3.0f};
    Intention dummyIntetion{Intention::FIND_ONESELF};

    comms.setPose(dummyPose, dummyTimestamp);
    comms.setWalkingTo(dummyWalk, dummyTimestamp);
    comms.setShootingTo(dummyShoot, dummyTimestamp);
    comms.setBallPosition(dummyBallPos, dummyTimestamp);
    comms.setBallVelocity(dummyBallVel, dummyTimestamp);
    comms.setIntention(dummyIntetion, dummyTimestamp);
}

int main(int argc, char** argv) {
    Communication c;

    // Temporary test
    fillDummyData(c);

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

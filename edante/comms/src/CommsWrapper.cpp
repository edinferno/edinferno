#include "ros/ros.h"
#include "std_msgs/String.h"
#include "comms/SPLStandardMessage.h"

#include "Communication.h"
#include "SPLStandardMessage.h"

typedef comms::SPLStandardMessage OurMsg;

OurMsg convertFrom(SPLStandardMessage msg) {
    OurMsg msg_; 
    msg_.pose[0] = msg.pose[0];
    msg_.pose[1] = msg.pose[1];
    msg_.pose[2] = msg.pose[2];
    msg_.walkingTo[0] = msg.walkingTo[0];
    msg_.walkingTo[1] = msg.walkingTo[1];
    msg_.shootingTo[0] = msg.shootingTo[0];
    msg_.shootingTo[1] = msg.shootingTo[1];
    msg_.ballAge = msg.ballAge;
    msg_.ball[0] = msg.ball[0];
    msg_.ball[1] = msg.ball[1];
    msg_.ballVel[0] = msg.ballVel[0];
    msg_.ballVel[1] = msg.ballVel[1];
    // Five is the maximum number of players
    for (int8_t i = 0; i < 5; i++) {
        msg_.suggestion[i] = msg.suggestion[i];
    }
    msg_.intention = msg.intention;
    msg_.averageWalkSpeed = msg.averageWalkSpeed;
    msg_.maxKickDistance = msg.maxKickDistance;
    msg_.currentPositionConfidence = msg.currentPositionConfidence;
    msg_.currentSideConfidence = msg.currentSideConfidence;
    msg_.numOfDataBytes = msg.numOfDataBytes;
    for (int8_t i = 0; i < msg.numOfDataBytes; i++) {
        msg_.data[i] = msg.data[i];
    }
    return msg_;
}

SPLStandardMessage convertTo(OurMsg msg) {
    SPLStandardMessage msg_; 
    msg_.pose[0] = msg.pose[0];
    msg_.pose[1] = msg.pose[1];
    msg_.pose[2] = msg.pose[2];
    msg_.walkingTo[0] = msg.walkingTo[0];
    msg_.walkingTo[1] = msg.walkingTo[1];
    msg_.shootingTo[0] = msg.shootingTo[0];
    msg_.shootingTo[1] = msg.shootingTo[1];
    msg_.ballAge = msg.ballAge;
    msg_.ball[0] = msg.ball[0];
    msg_.ball[1] = msg.ball[1];
    msg_.ballVel[0] = msg.ballVel[0];
    msg_.ballVel[1] = msg.ballVel[1];
    // Five is the maximum number of players
    for (int8_t i = 0; i < 5; i++) {
        msg_.suggestion[i] = msg.suggestion[i];
    }
    msg_.intention = msg.intention;
    msg_.averageWalkSpeed = msg.averageWalkSpeed;
    msg_.maxKickDistance = msg.maxKickDistance;
    msg_.currentPositionConfidence = msg.currentPositionConfidence;
    msg_.currentSideConfidence = msg.currentSideConfidence;
    msg_.numOfDataBytes = msg.numOfDataBytes;
    for (int8_t i = 0; i < msg.numOfDataBytes; i++) {
        msg_.data[i] = msg.data[i];
    }
    return msg_;
}

void fillDummyData(Communication& comms) {
    Timestamp dummyTimestamp{0};
    Pose2D dummyPose{0.2f, 0.4f, 0.8f};
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
    ros::Publisher pub = n.advertise<OurMsg>("comms/spl_std_msg", 1000);
    ros::Rate loopRate(5);
    while (ros::ok()) {
        pub.publish(convertFrom(c.getStandardMessage()));
        ros::spinOnce();
        loopRate.sleep();
    }
}

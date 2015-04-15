#include "SPLStandardMessage.h"

comms::SPLStandardMessage convertToROS(const SPLStandardMessage& msg) {
    comms::SPLStandardMessage msg_;

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
    for (int8_t i = 0; i < SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; i++) {
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

SPLStandardMessage convertFromROS(comms::SPLStandardMessage msg) {
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
    for (int8_t i = 0; i < SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; i++) {
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

std::ostream& operator<<(std::ostream& stream, const SPLStandardMessage& obj) {
    stream << "Pose: " << obj.pose[0] << " " << obj.pose[1] << " " << obj.pose[2] << std::endl;
    stream << "Walking To: " << obj.walkingTo[0] << " " << obj.walkingTo[1] << std::endl;
    stream << "Shooting To: " << obj.shootingTo[0] << " " << obj.shootingTo[1] << std::endl;

    stream << "Ball Age: " << obj.ballAge << std::endl;
    stream << "Ball Position: " << obj.ball[0] << " " << obj.ball[1] << std::endl;
    stream << "Ball Velocity: " << obj.ballVel[0] << " " << obj.ballVel[1] << std::endl;

    stream << "Suggestions: ";
    for (int i = 0; i < SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; i++) {
        stream << (int)obj.suggestion[i] << " ";
    }
    stream << std::endl;
    stream << "Intention: " << (int)obj.intention << std::endl;

    stream << "Average Walking Speed: " << obj.averageWalkSpeed << std::endl;
    stream << "Max Kick Distance: " << obj.maxKickDistance << std::endl;

    stream << "Current Position Confidence: " << (int)obj.currentPositionConfidence << std::endl;
    stream << "Current Side Confidence: " << (int)obj.currentSideConfidence << std::endl;

    stream << "Number of Data Bytes: " << obj.numOfDataBytes << std::endl;
    stream << "Data: " ;
    for (int i = 0; i < obj.numOfDataBytes; i++) {
      stream << std::hex << (int)obj.data[i] << " ";
    }
    stream << std::endl;
    return stream;
}

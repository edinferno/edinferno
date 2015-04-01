#include "Communication.h"

void Communication::setPose(Pose2D pose, Timestamp at) {
    stdmsg.pose[0] = pose.x;
    stdmsg.pose[1] = pose.y;
    stdmsg.pose[2] = pose.theta;
}

void Communication::setWalkingTo(Point2D point, Timestamp at) {
    stdmsg.walkingTo[0] = point.x;
    stdmsg.walkingTo[1] = point.y;
}

void Communication::unsetWalkingTo(Timestamp at) {
    /// If we don't want to move, give our position as our walking target.
    stdmsg.walkingTo[0] = stdmsg.pose[0];
    stdmsg.walkingTo[1] = stdmsg.pose[1];
}

void Communication::setShootingTo(Point2D point, Timestamp at) {
    stdmsg.shootingTo[0] = point.x;
    stdmsg.shootingTo[1] = point.y;
}

void Communication::unsetShootingTo(Timestamp at) {
    /// If we don't want to shoot, give our position as our shooting target.
    stdmsg.shootingTo[0] = stdmsg.pose[0];
    stdmsg.shootingTo[1] = stdmsg.pose[1];
}

void Communication::setBallPosition(Point2D point, Timestamp at) {
    stdmsg.ball[0] = point.x;
    stdmsg.ball[1] = point.y;
}

void Communication::setBallVelocity(Vector2D velocity, Timestamp at) {
    stdmsg.ballVel[0] = velocity.dx;
    stdmsg.ballVel[1] = velocity.dy;
}

void Communication::setIntention(Intention intention, Timestamp at) {
    stdmsg.intention = static_cast<int8_t>(intention);
}

SPLStandardMessage Communication::getStandardMessage() {

    /// The following properties of stdmsg are left as defualts:
    /// fallen, ballAge, suggestion, currentPositionConfidence, currentSideConfidence,
    /// numOfDataBytes, data
    /// This class can be extended by defining setters for them.

    return stdmsg;
}
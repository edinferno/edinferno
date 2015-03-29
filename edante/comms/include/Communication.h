/// Implementation of the Communication module from the Architecture Diagram.
///
/// Coordinate systems used
///  * Absolute coordinates (x, y) or (x, y, θ):
///     - x is the coordinate on Ox, which points from the center of the pitch
///       to the center of the opponent’s goal
///     - y is the coordinate on Oy, which is perpendicular to Ox and pointing
///       to the left when looking in the direction of Ox
///     - θ is a standard orientation in the [Ox, Oy] coordinate system
///       (increasing counter clockwise)
///     - linear measurements (x, y) are in mm; angular measurements θ are in
///       radians
///  * Relative coordinates (x, y) or (vx, vy):
///     - x is the coordinate on Ox, which points from the center of the robot
///       to the direction at which its body is facing
///     - y is the coordinate on Oy, which is perpendicular to Ox and pointing
///       to the left when looking in the direction of Ox (increasing counter
///       clockwise)
///     - spatial measurements (x, y) are in mm; speed measurements (vx, vy) are
///       in mm/ms
///  * Coordinated team time (CTT) - the temporal coordinate system for
///    timestamps:
///     - the time elapsed since the last synchronization of the clocks of the
///       team.
#ifndef EDANTE_COMMS_M
#define EDANTE_COMMS_M

#include "Time.h"
#include "Space.h"

#include "Mental.h"

#include "SPLStandardMessage.h"

class Communication {
private:
    SPLStandardMessage standardMessage;

public:
    Communication();

    /// Sets the pose of the robot on the field at the given point in time.
    ///  - The pose is given in absolute coordinates as (x, y, θ)
    void setPose(Pose2D pose, Timestamp at);
    /// Sets the position on the field that the robot plans to move to and the
    /// time at which the decision was taken.
    ///  - The point is in absolute coordinates as (x, y).
    void setWalkingTo(Point2D point, Timestamp at);
    /// Informs the “Communications” module that it was decided the robot will
    /// not move to another position on the field and provides the time at which
    /// the decision was taken.
    void unsetWalkingTo(Timestamp at);
    /// Sets the position on the field that the robot plans to shoot to and the
    /// time at which the decision was taken.
    ///  - The point is in absolute coordinates as (x, y).
    void setShootingTo(Point2D point, Timestamp at);
    /// Informs the “Communications” module that it was decided the robot will
    /// not shoot to another position on the field and provides the time at which
    /// the decision was taken.
    void unsetShootingTo(Timestamp at);
    /// Sets the position of the ball on the field and the time at which the
    /// observation was made.
    ///  - The point is in relative coordinates as (x, y).
    void setBallPosition(Point2D point, Timestamp at);
    /// Sets the speed of the ball on the field and the time at which the
    /// observation was made.
    ///  - The velocity vector is in relative coordinates as (vx, vy).
    void setBallVelocity(Vector2D velocity, Timestamp at);
    /// Sets the current intention of the robot and the time at which the
    /// decision was made.
    void setIntention(Intention intention, Timestamp at);
    /// Gives the latest constructed standard message.
    SPLStandardMessage getStandardMessage();
};

#define SPL_STANDARD_MESSAGE_STRUCT_DEFUALT_PLAYER_NUM 5
#define SPL_STANDARD_MESSAGE_STRUCT_DEFAULT_TEAM_NUM 17
#define SPL_STANDARD_MESSAGE_STRUCT_DEFAULT_AVERAGE_WALK_SPEED 5
#define SPL_STANDARD_MESSAGE_STRUCT_DEFAULT_MAX_KICK_DISTANCE 5

Communication::Communication() {
    standardMessage.playerNum = SPL_STANDARD_MESSAGE_STRUCT_DEFUALT_PLAYER_NUM;
    standardMessage.teamNum = SPL_STANDARD_MESSAGE_STRUCT_DEFAULT_TEAM_NUM;

    standardMessage.averageWalkSpeed = SPL_STANDARD_MESSAGE_STRUCT_DEFAULT_AVERAGE_WALK_SPEED;
    standardMessage.maxKickDistance = SPL_STANDARD_MESSAGE_STRUCT_DEFAULT_MAX_KICK_DISTANCE;
}

void Communication::setPose(Pose2D pose, Timestamp at) {
    standardMessage.pose[0] = pose.x;
    standardMessage.pose[1] = pose.y;
    standardMessage.pose[2] = pose.theta;
}

void Communication::setWalkingTo(Point2D point, Timestamp at) {
    standardMessage.walkingTo[0] = point.x;
    standardMessage.walkingTo[1] = point.y;
}

void Communication::unsetWalkingTo(Timestamp at) {
    /// If we don't want to move, give our position as our walking target.
    standardMessage.walkingTo[0] = standardMessage.pose[0];
    standardMessage.walkingTo[1] = standardMessage.pose[1];
}

void Communication::setShootingTo(Point2D point, Timestamp at) {
    standardMessage.shootingTo[0] = point.x;
    standardMessage.shootingTo[1] = point.y;
}

void Communication::unsetShootingTo(Timestamp at) {
    /// If we don't want to shoot, give our position as our shooting target.
    standardMessage.shootingTo[0] = standardMessage.pose[0];
    standardMessage.shootingTo[1] = standardMessage.pose[1];
}

void Communication::setBallPosition(Point2D point, Timestamp at) {
    standardMessage.ball[0] = point.x;
    standardMessage.ball[1] = point.y;
}

void Communication::setBallVelocity(Vector2D velocity, Timestamp at) {
    standardMessage.ballVel[0] = velocity.dx;
    standardMessage.ballVel[1] = velocity.dy;
}

void Communication::setIntention(Intention intention, Timestamp at) {
    standardMessage.intention = (int8_t) intention;
}

SPLStandardMessage Communication::getStandardMessage() {

    /// TODO: Set these
    /// standardMessage.fallen = ;

    /// standardMessage.ballAge = ;

    /// standardMessage.suggestion = ;

    /// standardMessage.currentPositionConfidence = ;
    /// standardMessage.currentSideConfidence = ;

    /// standardMessage.numOfDataBytes = ;
    /// standardMessage.data = ;

    return standardMessage;
}

#endif

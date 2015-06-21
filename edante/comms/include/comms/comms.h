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
#ifndef EDANTE_COMMS_H
#define EDANTE_COMMS_H

#include "SPLStandardMessage.h"

class Communication {
  private:
    SPLStandardMessage stdmsg;

  public:
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
    /// Returns a constant reference of the local struct to avoid the overhead
    /// of copying the entire struct.
    const SPLStandardMessage& getStandardMessage();
};

#endif

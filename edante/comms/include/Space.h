/// Defines structures representing space.
#ifndef EDANTE_SPACE_H
#define EDANTE_SPACE_H

/// Specifies a point in 2D space.
class Point2D {
public:
    /// Returns the x coordinate.
    double getX();
    /// Returns the y coordinate.
    double getY();
};


/// Specifies a vector in 2D space.
class Vector2D {
public:
    /// Returns the magnitude of the vector in the x direction.
    double getDX();
    /// Returns the magnitude of the vector in the y direction.
    double getDY();
};

/// Specifies a point and orientation in 2D space.
class Pose2D {
public:
    /// Returns the x coordinate.
    double getX();
    /// Returns the y coordinate.
    double getY();
    /// Returns the orientation θ.
    double getTheta();
};

#endif
